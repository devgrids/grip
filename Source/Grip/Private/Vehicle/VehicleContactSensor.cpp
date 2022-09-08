/**
*
* Wheel contact sensor implementation, use for wheels attached to vehicles.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Contact sensors provide information about nearest surface contacts for a wheel.
* They're paired for flippable vehicles so that we can detect contact but beneath
* and above any given wheel. They also provide suspension for standard vehicles
* and the hovering ability for antigravity vehicles.
*
***********************************************************************************/

#include "vehicle/vehiclecontactsensor.h"
#include "vehicle/basevehicle.h"
#include "gamemodes/basegamemode.h"

#pragma region VehicleContactSensors

/**
* Setup a new sensor.
***********************************************************************************/

void FVehicleContactSensor::Setup(ABaseVehicle* vehicle, int32 alignment, float side, float startOffset, float wheelWidth, float wheelRadius, float restingCompression)
{
	Vehicle = vehicle;
	Alignment = alignment;
	Side = side;
	WheelWidth = wheelWidth;
	WheelRadius = wheelRadius;
	RestingCompression = restingCompression;
	StartOffset = startOffset;
	SweepShape = FCollisionShape::MakeSphere(GetSweepWidth());
}

/**
* Sweeps along sensor direction to see if the suspension spring needs to compress.
* Returned collision time is normalized.
***********************************************************************************/

bool FVehicleContactSensor::GetCollision(UWorld* world, const FVector& start, const FVector& end, float& time, FHitResult& hitResult, bool estimate)
{
	FVector rayDirection = end - start;
	float lineLengthSqr = rayDirection.SizeSquared();
	FVector contactPointOnPlane = FVector::ZeroVector;

	check(start.ContainsNaN() == false);
	check(end.ContainsNaN() == false);
	check(rayDirection.ContainsNaN() == false);

	rayDirection.Normalize();

	if (estimate == true &&
		EstimateContact == true &&
		FMathEx::RayIntersectsPlane(start, rayDirection, EstimateContactPoint, EstimateContactNormal, contactPointOnPlane) == true)
	{
		// Estimation based on sensor / plane intersection. Assuming the last genuine contact
		// point is still valid the original point and normal of the intersection can be used
		// to describe a plane which we can calculate a new intersection with here.

		float distanceSqr = (contactPointOnPlane - start).SizeSquared();
		float sensorDistanceSqr = (end - start).SizeSquared();

		if (distanceSqr <= sensorDistanceSqr)
		{
			time = (EstimateDistance < KINDA_SMALL_NUMBER) ? 0.0f : EstimateTime * (FMath::Sqrt(distanceSqr) / EstimateDistance);

			return (time <= 1.0f);
		}
		else
		{
			return false;
		}
	}
	else
	{
		EstimateContact = false;

		if (lineLengthSqr > SMALL_NUMBER)
		{
			// Perform a sweep to determine nearest surface contacts.

			if (world->SweepSingleByChannel(hitResult, start, end, FQuat::Identity, ABaseGameMode::ECC_VehicleSpring, SweepShape, Vehicle->ContactSensorQueryParams) == true)
			{
				// If we detected a surface then determine the surface type.

				EGameSurface surfaceType = static_cast<EGameSurface>(UGameplayStatics::GetSurfaceType(HitResult));

				if (surfaceType != EGameSurface::Tractionless)
				{
					// If the surface isn't tractionless then process the result of the sweep.

					EstimateContactPoint = hitResult.ImpactPoint;
					EstimateContactNormal = hitResult.ImpactNormal;
					EstimateTime = hitResult.GetComponent() ? HitResult.Time : 1.f;

					check(hitResult.ImpactPoint.ContainsNaN() == false);
					check(hitResult.ImpactNormal.ContainsNaN() == false);
					check(rayDirection.ContainsNaN() == false);

					if (FVector::DotProduct(rayDirection, EstimateContactNormal) < 0.0f &&
						FMathEx::RayIntersectsPlane(start, rayDirection, EstimateContactPoint, EstimateContactNormal, EstimateContactPoint) == true)
					{
						// Setup estimation for sensor if the surface is geometrically suitable, it almost always is.

						EstimateContact = true;

						if (EstimateTime < KINDA_SMALL_NUMBER)
						{
							EstimateContactPoint = start;
						}
					}

					EstimateDistance = (EstimateContactPoint - start).Size();

					time = EstimateTime;
				}
			}
		}
	}

	return EstimateContact;
}

/**
* Computes new spring compression and force.
***********************************************************************************/

FVector FVehicleContactSensor::ComputeNewSpringCompressionAndForce(const FVector& endPoint, float deltaTime)
{
	// Get the compression of the suspension spring, and apply a modulating factor to accentuate movement.

	float compression = (endPoint - SensorPositionFromLength(WheelRadius + HoverDistance)).Size();

	// Make sure we don't over-react - never compress more than 80% of the wheel's radius.

	compression = FMath::Min(compression, (WheelRadius + HoverDistance) * 0.8f) / Vehicle->SpringEffect;

	check(FMath::IsNaN(compression) == false);

	// Get the difference between the compression on this frame and the last.

	float difference = (compression - Compression);

	// Compute a delta from that difference combined with the frame time.

	float delta = difference / deltaTime;

	float normalized = GetNormalizedCompression();

	check(FMath::IsNaN(normalized) == false);

	// Store the current compression for the next frame.

	Compression = compression;

	float newNormalized = GetNormalizedCompression();

	check(FMath::IsNaN(newNormalized) == false);

	if (CompressingHard == false)
	{
		CompressedHard = CompressingHard = ((newNormalized - normalized > 25.0f * deltaTime) && newNormalized >= 1.25f);
	}
	else
	{
		CompressedHard = false;
		CompressingHard = (newNormalized >= 1.25f);
	}

	// Now compute a response from the sensor direction, its stiffness and compression, along with some damping.

	float force = FMath::Clamp(-Vehicle->SpringStiffness * compression, -7500.0f, 7500.0f);

	return GetDirection() * (force - (Vehicle->SpringDamping * delta));
}

/**
* Calculate the nearest contact point of the spring in world space.
***********************************************************************************/

void FVehicleContactSensor::CalculateContactPoint(float deltaTime, UWorld* world, const FVector& startPoint, const FVector& direction, bool updatePhysics, bool estimate)
{
	// updatePhysics is only true if the sensor is part of the "active" set for a flippable vehicle -
	// either the top or bottom set depending on where we've detected a driving surface.

	FVector startPointOffset = startPoint + (direction * StartOffset * GetAlignment());

	StartPoint = startPointOffset;
	Direction = direction;

	// direction is the Z direction of the vehicle for reference.

	FVector end = SensorPositionFromLength(WheelRadius + HoverDistance);

	if (updatePhysics == true)
	{
		InContact = false;
		InEffect = false;
		NearestContactValid = false;

		float time = 1.0f;
		float sweepLength = GetSensorLength();
		FVector extent = SensorPositionFromLength(sweepLength);

		if (GetCollision(world, StartPoint, extent, time, HitResult, estimate) == true)
		{
			// If we have a collision with the scene geometry then compute the contact point
			// and other related data from it.

			SurfaceDistance = (FMath::Lerp(StartPoint, extent, time) - StartPoint).Size();
			SurfaceDistanceFromTire = 0.0f;

			float timeOffset = GetSweepWidth() / sweepLength;

			EndPoint = FMath::Lerp(StartPoint, extent, time + timeOffset);
			NearestContactPoint = EndPoint;
			NearestContactValid = true;
			NearestContactNormal = HitResult.ImpactNormal;

			float distance = (EndPoint - StartPoint).Size();

			InEffect = distance < WheelRadius + HoverDistance;
			InContact = distance < WheelRadius + HoverContactDistance;

			if (InContact == false)
			{
				EndPoint = end;
				SurfaceDistanceFromTire = distance - (WheelRadius + HoverContactDistance);

				CompressedHard = false;
				CompressingHard = false;
			}
		}
		else
		{
			// If no collision with the scene geometry then indicate so in our data.

			EndPoint = end;
			Compression = 0.0f;
			SurfaceDistance = 0.0f;
			SurfaceDistanceFromTire = -1.0f;
			CompressedHard = false;
			CompressingHard = false;
		}
	}
	else
	{
		// The opposite sensor is on the ground in this case, so this sensor cannot be.
		// Assume fully extended if this sensor is not part of the "active" set for the vehicle.

		InContact = false;
		InEffect = false;
		NearestContactValid = false;
		EndPoint = end;
		Compression = 0.0f;
		SurfaceDistance = 0.0f;
		SurfaceDistanceFromTire = -1.0f;
	}

	if (InContact == false)
	{
		NonContactTime += deltaTime;
	}
	else
	{
		NonContactTime = 0.0f;
	}
}

/**
* Do the regular update tick.
***********************************************************************************/

void FVehicleContactSensor::Tick(float deltaTime, UWorld* world, const FTransform& transform, const FVector& startPoint, const FVector& direction, bool updatePhysics, bool estimate, bool calculateIfUpward)
{
	if (calculateIfUpward == true ||
		GetAlignment() < 0.0f)
	{
		// updatePhysics is only true if the sensor is part of the "active" set for a flippable vehicle -
		// either the top or bottom set depending on where we've detected a driving surface.

		// direction is the Z direction of the vehicle for reference.

		CalculateContactPoint(deltaTime, world, startPoint, direction, updatePhysics, estimate);

		if (updatePhysics == true)
		{
			ForceToApply = FVector::ZeroVector;

			if (InEffect == true)
			{
				ForceToApply = ComputeNewSpringCompressionAndForce(EndPoint, deltaTime);

				check(ForceToApply.ContainsNaN() == false);
			}
			else
			{
				Compression = 0.0f;
				CompressedHard = false;
				CompressingHard = false;
			}
		}

		CompressionList.AddValue(Vehicle->GetVehicleClock(), GetNormalizedCompression());
	}
}

/**
* Apply the suspension spring force to the vehicle.
***********************************************************************************/

void FVehicleContactSensor::ApplyForce(const FVector& atPoint) const
{
	check(ForceToApply.ContainsNaN() == false);
	check(atPoint.ContainsNaN() == false);

	if (ForceToApply.IsNearlyZero() == false)
	{
		Vehicle->VehicleMesh->AddForceAtLocationSubstep(ForceToApply * Vehicle->GetPhysics().CurrentMass, atPoint);
	}
}

/**
* Get the direction of the sensor in world space.
***********************************************************************************/

FVector FVehicleContactSensor::GetDirection() const
{
	return Direction * Alignment;
}

/**
* Get the length of the ray casting down the sensor to detect driving surfaces.
***********************************************************************************/

float FVehicleContactSensor::GetSensorLength() const
{
	return (WheelRadius + Vehicle->HoverDistance) * 10.0f;
}

/**
* Get a normalized compression ratio of the suspension spring between 0 and 10, 1
* being resting under static weight.
***********************************************************************************/

float FVehicleContactSensor::GetNormalizedCompression(float value) const
{
	float restingCompression = (Vehicle->Antigravity == true) ? RestingCompression * 2.0f : RestingCompression;
	float ratio = value / (WheelRadius + HoverDistance);
	float compressionBreak = restingCompression / (WheelRadius + HoverDistance);

	check(FMath::IsNaN(ratio) == false);
	check(FMath::IsNaN(compressionBreak) == false);

	if (ratio <= KINDA_SMALL_NUMBER)
	{
		return 0.0f;
	}
	else if (ratio < compressionBreak)
	{
		return ratio / compressionBreak;
	}
	else
	{
		return 1.0f + ((ratio - compressionBreak) * 10.0f);
	}
}

/**
* Is the nearest contact point valid?
***********************************************************************************/

bool FVehicleContactSensor::HasNearestContactPoint(const FVector& wheelVelocity, float contactSeconds) const
{
	if (NearestContactValid == true)
	{
		if (InContact == true ||
			contactSeconds == 0.0f)
		{
			return true;
		}

		const FTransform& transform = Vehicle->VehicleMesh->GetPhysicsTransform();
		FVector contactLocal = transform.InverseTransformPosition(NearestContactPoint);
		FVector tireLocal = transform.InverseTransformPosition(GetRestingEndPoint() - (wheelVelocity * contactSeconds));

		// Because we're using the vehicle space, tireLocal.Z could be either positive or negative.

		if (tireLocal.Z >= 0.0f)
		{
			return (tireLocal.Z + (WheelRadius + HoverDistance) * 2.0f > contactLocal.Z);
		}
		else
		{
			return (tireLocal.Z - (WheelRadius + HoverDistance) * 2.0f < contactLocal.Z);
		}
	}

	return false;
}

/**
* Has the sensor detected a valid driving surface?
***********************************************************************************/

bool FVehicleContactSensor::HasValidDrivingSurface(const FVector& wheelVelocity, float contactSeconds) const
{
	EGameSurface surfaceType = GetGameSurface();

	return (HasNearestContactPoint(wheelVelocity, contactSeconds) == true) ? (surfaceType != EGameSurface::Tractionless) : false;
}

/**
* Get the amount of suspension spring extension (or offset of the wheel).
***********************************************************************************/

float FVehicleContactSensor::GetExtension() const
{
	float result = (((EndPoint - StartPoint).Size() - WheelRadius) + StartOffset) * GetAlignment();

	return result;
}

/**
* Get the width of the suspension sweep in cms.
***********************************************************************************/

float FVehicleContactSensor::GetSweepWidth() const
{
	// We scale the wheel ray-cast by 0.5 to get a radius, and then pinch
	// it in a bit more to avoid side collisions.

	const float widthScale = 0.5f;

	return (WheelWidth * 0.5f * widthScale);
}

/**
* Get the game surface of the last contact.
***********************************************************************************/

EGameSurface FVehicleContactSensor::GetGameSurface() const
{
	if (InContact == true &&
		GRIP_POINTER_VALID(HitResult.PhysMaterial) == true)
	{
		EPhysicalSurface surfaceType = UGameplayStatics::GetSurfaceType(HitResult);

		return (EGameSurface)surfaceType;
	}

	return EGameSurface::Num;
}

#pragma endregion VehicleContactSensors
