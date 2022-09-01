/**
*
* Vehicle physics implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Handle all of the physics-related activity of the vehicle. Most, if not all of
* this, will be executed via the SubstepPhysics function, and so via the physics
* sub-step, so we need to be very careful what we do here. All of the vehicle
* dynamics code can be found here.
*
***********************************************************************************/

#include "vehicle/vehiclephysics.h"
#include "vehicle/flippablevehicle.h"
#include "effects/drivingsurfacecharacteristics.h"
#include "pickups/shield.h"

#if WITH_PHYSX
#include "pxcontactmodifycallback.h"
#include "runtime/engine/private/physicsengine/physxsupport.h"
#endif // WITH_PHYSX

/**
* Do the regular physics update tick, for every sub-step.
*
* This is executed just prior to apply all forces and torques to this particular
* vehicle, though not necessarily before or after any other vehicles.
*
* Once all vehicles have been sub-stepped and forces / torques applied in this way
* the simulation itself is then stepped. Hence, any transforms within the physics
* system that are used in calculations here will be from the last physics sub-step.
*
* Consider this function here called in preparation for running this physics
* sub-step iteration.
*
* As the regular vehicle actor Tick is run PostPhysics you can do any cleanup work
* at the beginning of that Tick function, knowing that you'll be reading the most
* up-to-date information from the physics system.
***********************************************************************************/

void ABaseVehicle::SubstepPhysics(float deltaSeconds, FBodyInstance* bodyInstance)
{
	if (World == nullptr)
	{
		return;
	}

	// If the vehicle is idle-locked then clamp it by settings its location and orientation
	// and nullifying any velocity.

	if (VehicleMesh->UpdateIdleLock(true) == true)
	{
		VehicleMesh->SetPhysicsLocationAndQuaternionSubstep(VehicleMesh->GetIdleLocation(), VehicleMesh->GetIdleRotation());
		VehicleMesh->SetPhysicsLinearVelocitySubstep(FVector::ZeroVector);
		VehicleMesh->SetPhysicsAngularVelocityInRadiansSubstep(FVector::ZeroVector);
	}

	// Adjust the time passed to take into account custom time dilation for this actor.
	// This will always be 1 in this stripped version of the code, but it's important
	// that if you ever extend this to use CustomTimeDilation that we factor this in
	// right here.

	deltaSeconds *= CustomTimeDilation;
	deltaSeconds = FMath::Max(deltaSeconds, KINDA_SMALL_NUMBER);

	bool firstFrame = (Physics.Timing.TickCount <= 0);

	Physics.Timing.TickCount++;

	if (Physics.Timing.TickCount > 0)
	{
		Physics.Timing.TickSum += deltaSeconds;
	}

	// Grab a few things directly from the physics body and keep them in local variables,
	// sharing them around the update where appropriate.

	const FTransform& transform = VehicleMesh->GetPhysicsTransform();
	FQuat transformQuaternion = transform.GetRotation();
	FVector xdirection = transform.GetUnitAxis(EAxis::X);
	FVector ydirection = transform.GetUnitAxis(EAxis::Y);
	FVector zdirection = transform.GetUnitAxis(EAxis::Z);

	check(xdirection.ContainsNaN() == false);
	check(ydirection.ContainsNaN() == false);
	check(zdirection.ContainsNaN() == false);

	Physics.LastPhysicsTransform = Physics.PhysicsTransform;
	Physics.PhysicsTransform = transform;
	Physics.Direction = xdirection;

	// Get the world and local velocity in meters per second of the vehicle.

	FVector lastVelocity = Physics.VelocityData.Velocity;

	Physics.VelocityData.SetVelocities(VehicleMesh->GetPhysicsLinearVelocity(), VehicleMesh->GetPhysicsAngularVelocityInDegrees(), xdirection);

	// Calculate the acceleration vector of the vehicle in meters per second.

	Physics.VelocityData.AccelerationWorldSpace = (Physics.VelocityData.Velocity - lastVelocity) / deltaSeconds;
	Physics.VelocityData.AccelerationLocalSpace = transform.InverseTransformVector(Physics.VelocityData.AccelerationWorldSpace);
	Physics.DistanceTraveled += GetSpeedMPS() * deltaSeconds;
	Physics.AntigravitySideSlip = FMath::Max(0.0f, Physics.AntigravitySideSlip - (deltaSeconds * 0.333f));
	Physics.VelocityData.AngularVelocity = transform.InverseTransformVector(VehicleMesh->GetPhysicsAngularVelocityInDegrees());
	Physics.VehicleTBoned = FMath::Max(Physics.VehicleTBoned - deltaSeconds, 0.0f);
	Physics.SpringScaleTimer = FMath::Max(Physics.SpringScaleTimer - deltaSeconds, 0.0f);
	Physics.CurrentMass = Physics.StockMass;

	float brakePosition = 0.0f;
}

#if WITH_PHYSX
#if GRIP_ENGINE_PHYSICS_MODIFIED

/**
* Modify a collision contact.
*
* Be very careful here! This is called from the physics sub-stepping at the same
* time as other game code may be executing its normal ticks. Therefore, this code
* needs to be thread-safe and be able to handle re-entrancy.
***********************************************************************************/

bool ABaseVehicle::ModifyContact(uint32 bodyIndex, AActor* other, physx::PxContactSet& contacts)
{
	return false;
}

#endif // GRIP_ENGINE_PHYSICS_MODIFIED
#endif // WITH_PHYSX

/**
* Set the velocities and related data for the physics state.
***********************************************************************************/

void FPhysicsVelocityData::SetVelocities(const FVector& linearVelocity, const FVector& angularVelocity, const FVector& xdirection)
{
	check(linearVelocity.ContainsNaN() == false);
	check(angularVelocity.ContainsNaN() == false);

	Velocity = linearVelocity;
	VelocityDirection = Velocity;

	if (VelocityDirection.SizeSquared() < KINDA_SMALL_NUMBER)
	{
		VelocityDirection = xdirection;
	}
	else
	{
		VelocityDirection.Normalize();
	}

	Speed = Velocity.Size();

	// Set a maximum speed of 2,000 kph to help stop code breakages further down the line.

	float maxSpeed = 55555.555f;

	if (Speed > maxSpeed)
	{
		Speed = maxSpeed;
		Velocity = VelocityDirection * Speed;
	}

	DirectedSpeed = Speed;

	if (Speed > 10.0f)
	{
		DirectedSpeed *= FVector::DotProduct(xdirection, VelocityDirection);
	}
}

/**
* Construct a UTireFrictionModel structure.
***********************************************************************************/

UTireFrictionModel::UTireFrictionModel()
{
	LateralGripVsSlip.GetRichCurve()->AddKey(0, 0.0f);
	LateralGripVsSlip.GetRichCurve()->AddKey(2, 0.3f);
	LateralGripVsSlip.GetRichCurve()->AddKey(4, 0.5f);
	LateralGripVsSlip.GetRichCurve()->AddKey(8, 0.7f);
	LateralGripVsSlip.GetRichCurve()->AddKey(16, 1.0f);
	LateralGripVsSlip.GetRichCurve()->AddKey(32, 1.3f);

	LongitudinalGripVsSlip.GetRichCurve()->AddKey(0.0f, 0.0f);
	LongitudinalGripVsSlip.GetRichCurve()->AddKey(21.0f, 0.75f);
	LongitudinalGripVsSlip.GetRichCurve()->AddKey(28.0f, 0.8f);
	LongitudinalGripVsSlip.GetRichCurve()->AddKey(100.0f, 0.5f);

	LateralGripVsSpeed.GetRichCurve()->AddKey(0.0f, 128.0f);
	LateralGripVsSpeed.GetRichCurve()->AddKey(100.0f, 175.0f);
	LateralGripVsSpeed.GetRichCurve()->AddKey(200.0f, 285.0f);
	LateralGripVsSpeed.GetRichCurve()->AddKey(300.0f, 400.0f);
	LateralGripVsSpeed.GetRichCurve()->AddKey(400.0f, 525.0f);
	LateralGripVsSpeed.GetRichCurve()->AddKey(500.0f, 650.0f);
	LateralGripVsSpeed.GetRichCurve()->AddKey(600.0f, 775.0f);

	GripVsSuspensionCompression.GetRichCurve()->AddKey(0.0f, 0.0f);
	GripVsSuspensionCompression.GetRichCurve()->AddKey(0.5f, 0.8f);
	GripVsSuspensionCompression.GetRichCurve()->AddKey(1.0f, 1.0f);
	GripVsSuspensionCompression.GetRichCurve()->AddKey(2.0f, 2.0f);

	RearLateralGripVsSpeed.GetRichCurve()->AddKey(0.0f, 1.25f);
	RearLateralGripVsSpeed.GetRichCurve()->AddKey(500.0f, 1.25f);
}

/**
* Construct a UVehicleEngineModel structure.
***********************************************************************************/

UVehicleEngineModel::UVehicleEngineModel()
{
	GearPowerRatios.Emplace(0.75f);
	GearPowerRatios.Emplace(0.5f);
	GearPowerRatios.Emplace(0.75f);
}

/**
* Construct a USteeringModel structure.
***********************************************************************************/

USteeringModel::USteeringModel()
{
	FrontSteeringVsSpeed.GetRichCurve()->AddKey(0.0f, 1.0f);
	FrontSteeringVsSpeed.GetRichCurve()->AddKey(88.0f, 0.65f);
	FrontSteeringVsSpeed.GetRichCurve()->AddKey(166.0f, 0.4f);
	FrontSteeringVsSpeed.GetRichCurve()->AddKey(300.0f, 0.3f);
	FrontSteeringVsSpeed.GetRichCurve()->AddKey(450.0f, 0.25f);

	BackSteeringVsSpeed.GetRichCurve()->AddKey(0.0f, 1.0f);
	BackSteeringVsSpeed.GetRichCurve()->AddKey(50, 0.66f);
	BackSteeringVsSpeed.GetRichCurve()->AddKey(100.0f, 0.0f);
}
