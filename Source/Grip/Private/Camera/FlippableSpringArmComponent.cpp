/**
*
* Flippable spring arm implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Provides a spring arm for a camera that works well with flippable vehicles and
* contains a number of improvements over and above the standard engine spring arm.
* It doesn't care if the vehicle it's linked to isn't flippable, it doesn't matter.
*
* So the spring arm is used as a device to control where the camera sits behind a
* car when racing - showing the best view of the car to the player.
*
* It should always sit above and behind the car with respect to where the driving
* surface is. Defining "behind" is simple, but defining "above" is not quite so
* easy as the cars can flip over and the track can be upside down. Inevitably
* though, this will boil down to a world direction vector, which we can query
* directly from GetLaunchDirection on the vehicle.
*
* Positioning of the camera should be subject to smoothing as we don't want it
* violently moving from one frame to the next.
*
* The spring arm has several states of operation which we need to smoothly link to
* in order to avoid rough camera-work which is jarring to the player. Especially
* important here is the detection of the crashed state and our transition to and
* from it.
*
* We need to be more choosy about when transitioning back from airborne to normal
* states and be sure we really are in a normal state as erratic landings can give
* jarring camera-work at times.
*
* Crashed state means the vehicle is tumbling but in rough contact with the ground
* rather than being fully airborne (in which case tumbling isn't really an issue).
* It can also mean that the vehicle is jammed-up somewhere or close to a standing
* start but pointing in the wrong direction. It can also mean we're driving on a
* surface that is known to be invalid.
*
* We need camera-work here that handles all these situations well, and the
* transition out of it to normal driving can be delayed until we're definitely sure
* normal driving is being done, without any ill affect upon playability.
*
* Ensure all positional offsets / rotations are Schmitt-triggered and smoothed
* during the crash camera as it needs to be as reorienting and stable as possible
* for the player.
*
* So, prefer the race camera where possible.
* Airborne camera, following the velocity vector, when airborne.
* Crash camera, when the player is in trouble and in rough contact with the ground.
*
***********************************************************************************/

#include "camera/flippablespringarmcomponent.h"
#include "vehicle/flippablevehicle.h"
#include "system/mathhelpers.h"
#include "gamemodes/basegamemode.h"

const FName UFlippableSpringArmComponent::SocketName(TEXT("SpringEndpoint"));

/**
* Construct a flippable spring arm component.
***********************************************************************************/

UFlippableSpringArmComponent::UFlippableSpringArmComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;

	bAutoActivate = true;
	bTickInEditor = true;

	CameraOffsets.Emplace(FCameraOffset(-800.0f, 800.0f, 5.0f, 5.0f, 1.0f));
	CameraOffsets.Emplace(FCameraOffset(-600.0f, 600.0f, 5.0f, 5.0f, 1.0f));
	CameraOffsets.Emplace(FCameraOffset(-400.0f, 400.0f, 5.0f, 5.0f, 1.0f));
	CameraOffsets.Emplace(FCameraOffset(-200.0f, 200.0f, 5.0f, 5.0f, 0.0f));

#pragma region VehicleSpringArm

	CameraOffsetFrom = CameraOffsets[CameraOffsetIndex];
	CameraOffsetTo = CameraOffsets[CameraOffsetIndex];

	TargetLocation = CurrentLocation = GetComponentLocation();
	TargetRotation = CurrentRotation = GetComponentRotation();

	RelativeSocketLocation = FVector::ZeroVector;
	RelativeSocketRotation = FQuat::Identity;

	ClippingQueryParams.bReturnPhysicalMaterial = true;

	for (int32 i = 0; i < (int32)EFollowingMode::Num; i++)
	{
		FollowingModeVectors[i] = FVector::ZeroVector;
		SmoothedRotations[i] = GetComponentRotation();

		for (int32 j = 0; j < 2; j++)
		{
			TransitionRotations[i][j] = LastRotations[i][j] = FRotator::ZeroRotator;
		}
	}

#pragma endregion VehicleSpringArm

}

#pragma region VehicleSpringArm

/**
* Update the arm to the desired properties from a given transform.
***********************************************************************************/

void UFlippableSpringArmComponent::UpdateDesiredArmProperties(const FTransform& vehicleTransform, FRotator vehicleRotation, float deltaSeconds)
{
	UpdateDesiredArmProperties(vehicleTransform, vehicleRotation, true, true, true, deltaSeconds);
}

/**
* Update the arm to the desired properties.
***********************************************************************************/

void UFlippableSpringArmComponent::UpdateDesiredArmProperties(const FTransform& vehicleTransform, FRotator vehicleRotation, bool doClippingCheck, bool doLocationLag, bool doRotationLag, float deltaSeconds)
{
	// We pass in vehicleRotation separately because we want a rotation that doesn't suffer
	// from asymptotes when the direction is very close to the vertical.

	check(GetRelativeLocation() == FVector::ZeroVector);

	ABaseVehicle* vehicle = Cast<ABaseVehicle>(GetAttachmentRootActor());
	UGlobalGameState* gameState = UGlobalGameState::GetGlobalGameState(GetWorld());

	if (vehicle != nullptr &&
		gameState != nullptr)
	{
		// If we've not attached to our parent vehicle for collision queries then do that now.

		if (ClippingQueryParams.GetIgnoredActors().Num() == 0)
		{
			ClippingQueryParams.AddIgnoredActor(vehicle);
		}

		bool airborne = vehicle->IsAirborne(true);

		if (Airborne != airborne)
		{
			Airborne = airborne;
			ContactModeTime = 0.0f;
			NoAirborneContactTime = 0.0f;
		}
		else
		{
			ContactModeTime += deltaSeconds;
		}

		FVector vehicleHeading = vehicle->GetTargetHeading();

		if (TargetVehicleHeading.IsZero() == true)
		{
			TargetVehicleHeading = vehicleHeading;
		}
		else
		{
			float smoothHeading = FMathEx::GetSmoothingRatio(0.95f, deltaSeconds);

			// TODO: Try to effect an ease-out/in interpolation between any large changes in direction
			//       which are normally attributable to vehicle spline swapping.

			TargetVehicleHeading = FMath::Lerp(vehicleHeading, TargetVehicleHeading, smoothHeading);
			TargetVehicleHeading.Normalize();
		}

		float enterCrashCameraSpeed = 150.0f;
		float speedKPH = vehicle->GetSpeedKPH();
		float shortModeTransitionTime = ModeTransitionTime * 0.25f;
		FVector vehicleDirection = vehicle->GetFacingDirection();
		FVector vehicleVelocityDirection = vehicle->GetVelocityOrFacingDirection();
		bool hasSmashedIntoSomething = vehicle->HasSmashedIntoSomething(enterCrashCameraSpeed);
		float headingAngleDifference = FVector::DotProduct(vehicleDirection, TargetVehicleHeading);
		float forwardSpeedKPH = speedKPH * FMath::Max(0.0f, FVector::DotProduct(TargetVehicleHeading, vehicleVelocityDirection));

		bool useCrashCamera = false;

		// Determine mode at this moment.

		EFollowingMode followingMode = EFollowingMode::Normal;

		// TODO: Question about going from crashed to airborne mode - should we do this?

		if (airborne == true &&
			speedKPH > 100.0f &&
			hasSmashedIntoSomething == false)
		{
			// Only try the airborne camera if we're not going too slowly and haven't just crashed.

			followingMode = EFollowingMode::Airborne;
		}
		else if (useCrashCamera == true)
		{
			// An initial, simple implementation might have the pitch / yaw angle difference kick in the
			// crash camera. And only when some vehicle speed and angle difference being within range
			// over time should we merge back to the normal camera again. I think this will cover the
			// majority of crash cases and we can work on refining it a little later.

			if (FollowingMode == EFollowingMode::Crashed)
			{
				// If we're already crashed then see if we should remain so.

				if ((hasSmashedIntoSomething == true) ||
					(forwardSpeedKPH < 100.0f || headingAngleDifference < FMathEx::DegreesToDotProduct(20.0f)))
				{
					followingMode = EFollowingMode::Crashed;
				}
			}
			else
			{
				// If we're not already crashed then see if we have become that.

				if ((hasSmashedIntoSomething == true) ||
					(speedKPH < enterCrashCameraSpeed && headingAngleDifference < FMathEx::DegreesToDotProduct(45.0f)))
				{
					// We're off heading by at least 45 degrees or have just crashed.

					followingMode = EFollowingMode::Crashed;
				}
			}
		}

		// Now apply some gate to any mode changes as we don't want hysteresis.

		if (followingMode != FollowingMode)
		{
			bool allowModeTransition = true;

			if (FollowingModeTime < ThisModeTransitionTime)
			{
				allowModeTransition = false;
			}

			if (allowModeTransition == true)
			{
				switch (FollowingMode)
				{
				case EFollowingMode::Crashed:

					// Got to be non-crashed for at least a couple of seconds before we'll allow
					// transition out of this mode.

					allowModeTransition = (ToFollowingModeTime >= 2.0f);

					break;

				default:

					// Going from airborne or normal, allow a switch as long as we've been active for a
					// half second already.

					allowModeTransition = (FollowingModeTime >= shortModeTransitionTime);

					break;
				}
			}

			if (allowModeTransition == false)
			{
				// If we're not allowed out of this state then use the current following mode.

				followingMode = FollowingMode;
			}
		}

		// Handle the transition between different target locations.

		FCameraOffset targetLength;

		targetLength.InterpEaseInOut(CameraOffsetFrom, CameraOffsetTo, CameraOffsetTime, 2.0f);

		CameraOffsetTime = FMath::Min(CameraOffsetTime + deltaSeconds * 2.0f, 1.0f);

		// We have different levels of lag depending on where the camera is relative to the vehicle.

		float closeInRatio = 1.0f;
		bool isCockpitView = vehicle->IsCockpitView();

		if (isCockpitView == true)
		{
			// No lag in cockpit view.

			closeInRatio = 0.0f;
		}
		else
		{
			// Compute the lag of the target length from the vehicle.

			closeInRatio = CameraOffsetFrom.LagRatio * (1.0f - CameraOffsetTime) + CameraOffsetTo.LagRatio * CameraOffsetTime;
			closeInRatio = FMath::Clamp(closeInRatio, 0.0f, 1.0f);
		}

		if (SmoothingReset == true ||
			closeInRatio < KINDA_SMALL_NUMBER)
		{
			closeInRatio = 0.0f;
		}

		if (OwnerIsBeingWatched() == false)
		{
			// Don't to clipping checks for no reason.

			doClippingCheck = false;
		}

		bool flipped = vehicle->IsFlipped();

		if (closeInRatio == 0.0f ||
			IsBumperView() == true)
		{
			// If we're zoomed right in, then flip the camera only when the car in in contact with the
			// ground (normally it's a best guess depending on where the closest ground point is).

			flipped = vehicle->IsFlippedAndWheelsOnGround();

			if (IsBumperView() == true &&
				doClippingCheck == true)
			{
				FHitResult result;
				float zOffset = targetLength.ZOffset;
				FVector armRoot = GetComponentTransform().TransformPosition(FVector(targetLength.XOffset, 0.0f, 0.0f));
				FVector armEnd = GetComponentTransform().TransformPosition(FVector(targetLength.XOffset, 0.0f, (flipped == true) ? -zOffset : zOffset));

				if (GetWorld()->SweepSingleByChannel(result, armRoot, armEnd, FQuat::Identity, ABaseGameMode::ECC_VehicleCamera, FCollisionShape::MakeSphere(ProbeSize), ClippingQueryParams) == true &&
					(EGameSurface)UGameplayStatics::GetSurfaceType(result) != EGameSurface::Field)
				{
					flipped ^= true;
					armEnd = GetComponentTransform().TransformPosition(FVector(targetLength.XOffset, 0.0f, (flipped == true) ? -zOffset : zOffset));

					if (GetWorld()->SweepSingleByChannel(result, armRoot, armEnd, FQuat::Identity, ABaseGameMode::ECC_VehicleCamera, FCollisionShape::MakeSphere(ProbeSize), ClippingQueryParams) == true &&
						(EGameSurface)UGameplayStatics::GetSurfaceType(result) != EGameSurface::Field)
					{
						flipped ^= true;
					}
				}
			}
		}

		if (closeInRatio == 0.0f)
		{
			// We have a fixed camera point track the vehicle directly.

			doClippingCheck = false;
		}

		if (closeInRatio == 0.0f ||
			SmoothingReset == true ||
			vehicle->HasRespawnLanded() == false ||
			CameraOffsetIndex == CameraOffsets.Num() - 1)
		{
			followingMode = EFollowingMode::Normal;
		}

		// Set the mode once we're sure about it.

		if (ToFollowingMode != followingMode)
		{
			ToFollowingModeTime = 0.0f;
			ToFollowingMode = followingMode;
		}
		else
		{
			ToFollowingModeTime += deltaSeconds;
		}

		FRotator velocityDirection = vehicle->GetVelocityOrFacingDirection().Rotation();

		if (FollowingMode != followingMode)
		{
			FromFollowingMode = FollowingMode;

			FollowingModeTime = 0.0f;
			FollowingMode = followingMode;

			if (FollowingMode == EFollowingMode::Airborne)
			{
				AirborneVehicleHeading = velocityDirection;
			}

			// #TODO: Set the transition time to take into account the disparity between the
			// from and to rotations, with small differences taking little time.

			if (FromFollowingMode == EFollowingMode::Normal &&
				FollowingMode == EFollowingMode::Airborne)
			{
				ThisModeTransitionTime = shortModeTransitionTime;
			}
			else if (FromFollowingMode == EFollowingMode::Airborne &&
				FollowingMode == EFollowingMode::Normal)
			{
				// Use a secondary merging of the airborne direction vector if we're going fast enough.
				// Otherwise use the standard interpolation between modes.

				AirToGroundTime = (speedKPH > 100.0f) ? 0.0f : 10.0f;
				ThisModeTransitionTime = (AirToGroundTime == 0.0f) ? 0.0f : shortModeTransitionTime;
			}
			else
			{
				// Going in to or out of crashed mode.

				ThisModeTransitionTime = ModeTransitionTime;

				if (FollowingMode == EFollowingMode::Crashed &&
					hasSmashedIntoSomething == true)
				{
					// If we have crashed sharply then don't hang about in the transition.

					ThisModeTransitionTime *= 0.333f;
				}
			}
		}
		else
		{
			FollowingModeTime += deltaSeconds;
		}

		FVector launchDirection = vehicle->GetLaunchDirection();

		if (SmoothingReset == true)
		{
			AirToGroundTime = 10.0f;
			LaunchDirectionFlipTime = 0.0f;
			LaunchDirection = launchDirection;
			FollowingModeTime = ThisModeTransitionTime;
		}

		// We only apply speed shake when the car is on the ground, but fade it in / out
		// when transitioning between grounded and airborne.

		if (airborne == true)
		{
			SpeedShakeTimer += deltaSeconds;
		}
		else
		{
			SpeedShakeTimer -= deltaSeconds;
		}

		SpeedShakeTimer = FMath::Clamp(SpeedShakeTimer, 0.0f, 1.0f);

		// Calculate what the desired rotation should be.

#pragma region VehicleContactSensors

		bool surfaceDirectionValid = vehicle->IsSurfaceDirectionValid(1.0f);

#pragma endregion VehicleContactSensors

		if (flipped == true)
		{
			// We want to be on the side of the vehicle facing away from the ground so we have
			// to take that into account here.

			vehicleRotation.Roll = FRotator::NormalizeAxis(vehicleRotation.Roll + 180.0f);
		}

		// Calculate the target rotation for the camera.

		// We only track rotation if we're on the ground.
		// If we're airborne then we want to be looking down the velocity vector (see below).

		Rotations[(int32)EFollowingMode::Normal] = vehicleRotation;

		// If we're airborne then look in the direction of the velocity vector.

		// When velocity is low or even reverses due to landing and hitting something then we need to
		// do something with the rotation here. Soon enough we'll either be in the normal or crashed
		// camera mode but until then we need the rotation to do something predictable. So for now,
		// we just remember the last good rotation and use that until it recovers.

		bool velocityDirectionValid = false;

		if (speedKPH >= 100.0f)
		{
			velocityDirectionValid = true;
			AirborneVehicleHeading = velocityDirection;
		}

		Rotations[(int32)EFollowingMode::Airborne] = AirborneVehicleHeading;

#pragma region VehicleContactSensors

		bool hasSomeIdeaOfSurface = vehicle->GetWheels().HasSurfaceDirection();
		bool hasGoodIdeaOfSurface = vehicle->GetWheels().HasConfidentSurfaceDirection();

#pragma endregion VehicleContactSensors

		if (Airborne == true &&
			ContactModeTime != 0.0f &&
			hasSomeIdeaOfSurface == false)
		{
			NoAirborneContactTime += deltaSeconds;
		}

		if ((hasGoodIdeaOfSurface == true || NoAirborneContactTime == 0.0f) &&
			velocityDirectionValid == true)
		{
			// If we're in some sort of contact with the ground recently then try to preserve the roll
			// of the camera towards the ground normal.

			// #TODO: Large or rapid changes to this roll can result in jarring rotation, especially
			// when merging the rotations in the next if statement below.

			Rotations[(int32)EFollowingMode::Airborne].Roll = vehicleRotation.Roll;
		}

		// Handle air to ground recovery rotation interpolation.

		const float airToGroundRecoveryTime = 0.25f;
		const float airToGroundTransitionTime = 1.0f;

		// #TODO: Check out this rotation merging here as it appears to over-rotate at times, often
		// going a full 360 degrees when it should just rotate a little the other way.

		if (AirToGroundTime < airToGroundRecoveryTime)
		{
			// Recovering from air to ground so reset the normal mode to use the airborne straight.

			// This just works better until the normal driving has had chance to resume nicely and
			// results in a smoother transition.

			Rotations[(int32)EFollowingMode::Normal] = Rotations[(int32)EFollowingMode::Airborne];
			SmoothedRotations[(int32)EFollowingMode::Normal] = SmoothedRotations[(int32)EFollowingMode::Airborne];

			if (vehicle->IsGrounded() == true)
			{
				AirToGroundTime = airToGroundRecoveryTime;
			}

			if (AirToGroundTime + deltaSeconds >= airToGroundRecoveryTime)
			{
				// Setup the initial from rotations for a transition so that we are able to keeping transitioning
				// in the correct direction regardless of which is closest.

				for (int32 i = 0; i < (int32)EFollowingMode::Num; i++)
				{
					TransitionRotations[i][1] = Rotations[i].GetNormalized();
					LastRotations[i][1] = Rotations[i];
				}

				ModifyRotationBasis(TransitionRotations[(int32)EFollowingMode::Airborne][1], TransitionRotations[(int32)EFollowingMode::Normal][1]);
			}
		}
		else if (AirToGroundTime < airToGroundRecoveryTime + airToGroundTransitionTime)
		{
			for (int32 i = 0; i < (int32)EFollowingMode::Num; i++)
			{
				TransitionRotations[i][1] += FMathEx::GetSignedDegreesDifference(LastRotations[i][1], Rotations[i]);
				LastRotations[i][1] = Rotations[i];
			}

			float followingRatio = (AirToGroundTime - airToGroundRecoveryTime) / airToGroundTransitionTime;

			followingRatio = FMath::InterpEaseInOut(0.0f, 1.0f, followingRatio, 2.0f);

			Rotations[(int32)EFollowingMode::Normal] = FMathEx::RInterpToRaw(TransitionRotations[(int32)EFollowingMode::Airborne][1], TransitionRotations[(int32)EFollowingMode::Normal][1], followingRatio);
		}

		AirToGroundTime += deltaSeconds;

		// If we're crashed then look down the track with the camera offset in the launch direction
		// as normal. We'll correct to have the camera always have its head-up a little later in the code.

		Rotations[(int32)EFollowingMode::Crashed] = TargetVehicleHeading.Rotation();

		if (FVector::DotProduct(LaunchDirection, launchDirection) < 0.0f)
		{
			LaunchDirectionFlipTime = 0.0f;
		}
		else
		{
			LaunchDirectionFlipTime += deltaSeconds;
		}

		LaunchDirection = launchDirection;

		FollowingModeVectors[(int32)EFollowingMode::Crashed] = launchDirection;

		const float flipRollDuration = 0.5f;

		if (LaunchDirectionFlipTime < flipRollDuration)
		{
			float launchRollRatio = FMath::InterpEaseOut(0.0f, 1.0f, FMath::Min(LaunchDirectionFlipTime / flipRollDuration, 1.0f), 2.0f);

			FollowingModeVectors[(int32)EFollowingMode::Crashed] = FMath::Lerp(launchDirection * -1.0f, launchDirection, launchRollRatio);
		}

		if (doRotationLag == true)
		{
			// Calculate smoothed rotations for each of the modes so we always have these
			// to dynamically transition between on following-mode changes.

			for (int32 i = 0; i < (int32)EFollowingMode::Num; i++)
			{
				// Apply 'lag' to rotation.

				YawLagRatio = CameraYawLagRatio;
				PitchLagRatio = CameraPitchLagRatio;
				RollLagRatio = CameraRollLagRatio;

				float speedRatio = 0.80f;

				if (RollLagRatio > speedRatio &&
					SpeedRollTimer > 0.0f)
				{
					// We want to change the roll position quickly as it's just switched massively and
					// we're likely to encounter clipping problems if we don't get there fast.
					// SpeedRollTimer normally starts at 1 and drops to 0.

					float rollRatio = FMath::Sin(SpeedRollTimer * PI);

					// speedRatio is the rate at which we want to roll quickly.

					RollLagRatio = (speedRatio * rollRatio) + (RollLagRatio * (1.0f - rollRatio));
				}

				if (SmoothingReset == true)
				{
					// We've been asked to reset, so don't do any lag here.

					YawLagRatio = 0.0f;
					PitchLagRatio = 0.0f;
					RollLagRatio = 0.0f;

					Rotations[i].Roll = 0.0f;
				}

				if (i == (int32)EFollowingMode::Crashed)
				{
					YawLagRatio = 0.0f;
					PitchLagRatio = 0.0f;
					RollLagRatio = 0.0f;
				}

				// Guess what, when driving along a wall pitch and yaw are transposed!

				// We've a bit of a hack here to separate that out and apply the correct lag to each
				// component any given situation but I wouldn't say it was an inspired solution.

				FRotator r0 = Rotations[i];
				FRotator r1 = SmoothedRotations[i];
				float rd = FMathEx::GetUnsignedDegreesDifference(r0.Roll, r1.Roll);
				float pd = FMathEx::GetUnsignedDegreesDifference(r0.Pitch, r1.Pitch);
				float yawRatio = closeInRatio;
				float rollRatio = FMath::Lerp(1.0f, FMath::Max(closeInRatio, 0.5f), FMathEx::GetRatio(rd, 25.0f, 50.0f));
				float pitchRatio = FMath::Lerp(1.0f, FMath::Max(closeInRatio, 0.5f), FMathEx::GetRatio(pd, 25.0f, 50.0f));
				float rollSwap = FMath::Abs(r1.Roll); if (rollSwap > 90.0f) rollSwap = 180.0f - rollSwap;
				float pitchYawSwap = FMathEx::GetRatio(rollSwap, 0.0f, 90.0f);

				if (rd > 90.0f)
				{
					rollRatio = closeInRatio;
				}

				// Now that we've calculated all of the lag ratios, calculate the smooth rotation from
				// the last rotation to the desired one based on those ratios.

				SmoothedRotations[i] = FMathEx::GetSmoothedRotation(r1, r0, deltaSeconds, YawLagRatio * FMath::Lerp(yawRatio, pitchRatio, pitchYawSwap), PitchLagRatio * FMath::Lerp(pitchRatio, yawRatio, pitchYawSwap), RollLagRatio * rollRatio);
			}
		}
		else
		{
			for (int32 i = 0; i < (int32)EFollowingMode::Num; i++)
			{
				SmoothedRotations[i] = Rotations[i];
			}
		}

		// Calculate the transition rotations to preserve rotation direction through interpolation.

		float followingRatio = GetFollowingTransitionRatio();
		bool transitioning = (followingRatio < 1.0f - KINDA_SMALL_NUMBER);

		if (transitioning == false ||
			FollowingModeTime == 0.0f ||
			TransitionRotations[0][0] == FRotator::ZeroRotator)
		{
			// Setup the initial from rotations for a transition so that we are able to keeping transitioning
			// in the correct direction regardless of which is closest.

			for (int32 i = 0; i < (int32)EFollowingMode::Num; i++)
			{
				TransitionRotations[i][0] = SmoothedRotations[i].GetNormalized();
				LastRotations[i][0] = SmoothedRotations[i];
			}

			ModifyRotationBasis(TransitionRotations[(int32)FromFollowingMode][0], TransitionRotations[(int32)FollowingMode][0]);
		}
		else
		{
			for (int32 i = 0; i < (int32)EFollowingMode::Num; i++)
			{
				TransitionRotations[i][0] += FMathEx::GetSignedDegreesDifference(LastRotations[i][0], SmoothedRotations[i]);
				LastRotations[i][0] = SmoothedRotations[i];
			}
		}

		if (transitioning == false)
		{
			TargetRotation = TransitionRotations[(int32)FollowingMode][0].GetNormalized();
		}
		else
		{
			if (FollowingMode != EFollowingMode::Crashed &&
				FromFollowingMode != EFollowingMode::Crashed)
			{
				followingRatio = FMath::InterpEaseOut(0.0f, 1.0f, followingRatio, 3.0f);
			}
			else
			{
				followingRatio = FMath::InterpEaseInOut(0.0f, 1.0f, followingRatio, 2.0f);
			}

			TargetRotation = FMathEx::RInterpToRaw(TransitionRotations[(int32)FromFollowingMode][0], TransitionRotations[(int32)FollowingMode][0], followingRatio);
		}

		if (FollowingModeTime > ThisModeTransitionTime)
		{
			FromFollowingMode = FollowingMode;
		}

		// So TargetRotation is now the rotation in world space that we wish the camera to
		// use, which will now be processed further with smoothing etc.

		// Both rotations are normalized to -180 to +180.

		if (surfaceDirectionValid == true &&
			FMathEx::GetUnsignedDegreesDifference(Rotations[(int32)EFollowingMode::Normal].Roll, SmoothedRotations[(int32)EFollowingMode::Normal].Roll, true) > 120.0f)
		{
			// If we're trying roll more than 120.0 degrees then let's do it quickly.
			// This happens when the car flips with respect to the "ground" normally.

			if (SpeedRollTimer < 0.5f)
			{
				// If we're not speed-rolling then we will start with 1.
				// If we are then we'll end up with something between 0.5 and 1.

				SpeedRollTimer = 0.5f + (0.5f - SpeedRollTimer);
			}
		}

		// We add in the forced offset afterwards as we don't want that aspect to be smoothed.

		UpdateRotationOffset(deltaSeconds, vehicle, closeInRatio);

		// This offset is calculated in local space, but the addition works in world space.

		CurrentRotation = TargetRotation + RotationOffset;

		// If the game has finished for the player then look back at the car from the furthest
		// camera position rotated around to the front rather than the normal rear position.

		if (vehicle->GetRaceState().PlayerCompletionState == EPlayerCompletionState::Complete)
		{
			if (vehicle->IsCinematicCameraActive(false) == false)
			{
				OrbitHor = 180.0f;
				closeInRatio = CameraOffsets[0].LagRatio;
				targetLength = CameraOffsets[1];
			}
		}
		else
		{
			OrbitHor = CurrentUserYawAngle;
			OrbitVer = 0.0f;
		}

		DownAngle = FMath::Lerp(targetLength.MaxDownAngle, targetLength.MinDownAngle, FieldOfViewProportion);

		if (isCockpitView == true)
		{
			// If we're in cockpit view then put the camera 50cm above the center point and 100cm back from the front of the vehicle.

			targetLength.XOffset = vehicle->GetBoundingExtent().X - 100.0f;
			targetLength.XOffset = 0.0f;

			targetLength.XOffset = FMath::Lerp(targetLength.XOffset, -targetLength.XOffset, FMathEx::GetRatio(FMath::Abs(OrbitHor), 0.0f, 180.0f));

			TargetLocation = GetComponentTransform().TransformPosition(FVector(targetLength.XOffset, 0.0f, (flipped == true) ? -50.0f : 50.0f));

			DownAngle = 0.0f;
		}
		else if (closeInRatio == 0.0f)
		{
			// We have a fixed camera point.

			TargetLocation = GetComponentTransform().TransformPosition(FVector(targetLength.XOffset, 0.0f, (flipped == true) ? -targetLength.ZOffset : targetLength.ZOffset));
		}
		else
		{
			// Get the anchor point on the vehicle the spring arm is notionally connected to.
			// In fact, this is just the center of the vehicle as GetRelativeLocation() is a zero vector.

			FVector attachmentRoot = vehicleTransform.TransformPositionNoScale(GetRelativeLocation());
			FVector speedShakeOffset = FVector::ZeroVector;

			if (SpeedShakeAmount > 0.0f)
			{
				float deltaTime = deltaSeconds / SpeedShakeFrequency;

				speedShakeOffset.Y = FMathEx::UpdateOscillator(SpeedShakeX, vehicle->PerlinNoise, deltaTime * SpeedShakeSpeed) * 0.3f;
				speedShakeOffset.Z = FMathEx::UpdateOscillator(SpeedShakeY, vehicle->PerlinNoise, deltaTime * SpeedShakeSpeed);

				speedShakeOffset *= SpeedShakeAmount * SpeedShakeAmplitude * closeInRatio * (1.0f - SpeedShakeTimer);
			}

			// Calculate the arm root based on closest camera point and the flipped state of the vehicle.

			FVector armOffset = FVector::ZeroVector;

			if (transitioning == false)
			{
				armOffset = MakeArmOffset(targetLength, CurrentRotation, FollowingMode, true);
			}
			else
			{
				// Complicated interpolation technique due to the crashed state having a non
				// rotation-interpolated vertical offset to the spring arm.

				armOffset = MakeArmOffset(targetLength, CurrentRotation, FromFollowingMode, false);

				const FVector armOffset0 = MakeArmOffset(targetLength, CurrentRotation, FromFollowingMode, true) - armOffset;
				const FVector armOffset1 = MakeArmOffset(targetLength, CurrentRotation, FollowingMode, true) - armOffset;

				FVector verticalOffset = FMath::Lerp(armOffset0, armOffset1, followingRatio);

				verticalOffset.Normalize();
				verticalOffset *= FMath::Lerp(armOffset0.Size(), armOffset1.Size(), followingRatio);

				armOffset += verticalOffset;
			}

			FVector armRoot = MakeArmRoot(attachmentRoot, armOffset, flipped);

			for (int32 i = 0; i < 4; i++)
			{
				TargetLocation = attachmentRoot + armOffset;

				// TargetLocation is now where the camera ought to be.

				// Apply the camera shake.

				TargetLocation += CurrentRotation.RotateVector(speedShakeOffset) * closeInRatio;

				// Do a sweep to ensure we are not penetrating the world.

				if (doClippingCheck == true)
				{
					FHitResult result;

					FVector toCamera = TargetLocation - armRoot;
					FVector toDirection = toCamera;
					float toSize = toCamera.Size();

					toDirection.Normalize();

					float padding = ProbeSize * 4.0f;
					FVector armEnd = armRoot + toCamera + (toDirection * padding);

					if (GetWorld()->SweepSingleByChannel(result, armRoot, armEnd, FQuat::Identity, ABaseGameMode::ECC_VehicleCamera, FCollisionShape::MakeSphere(ProbeSize), ClippingQueryParams) == true &&
						(EGameSurface)UGameplayStatics::GetSurfaceType(result) != EGameSurface::Field)
					{
						float difference = (armEnd - armRoot).Size();
						float distance = (difference * result.Time) - padding;

						distance = FMath::Max(distance, 0.0f);

						TargetLocation = FMath::Lerp(armRoot, armEnd, distance / toSize);
					}
					else
					{
						break;
					}
				}
				else
				{
					break;
				}
			}

			if (doClippingCheck == true)
			{
				ClipAgainstVehicles(armRoot, TargetLocation);
			}

			// Handle smoothing of the clipped distance of the camera to its parent.

			FVector toTarget = TargetLocation - attachmentRoot;
			FVector toArmRoot = armRoot - attachmentRoot;
			float clippedDistance = toTarget.Size();
			float armRootDistance = toArmRoot.Size();

			if (LastClippingDistance != 0.0f &&
				LastClippingDistance < armRootDistance)
			{
				LastClippingDistance = armRootDistance;
			}

			if (LastClippingDistance == 0.0f ||
				clippedDistance < LastClippingDistance)
			{
				LastClippingDistance = clippedDistance;
			}
			else
			{
				float ratio = FMathEx::GetSmoothingRatio(0.975f, deltaSeconds);

				LastClippingDistance = (LastClippingDistance * ratio) + (clippedDistance * (1.0f - ratio));

				float scale = LastClippingDistance / clippedDistance;

				TargetLocation = attachmentRoot + (toTarget * scale);
			}
		}

		CurrentLocation = TargetLocation;

		SmoothingReset = false;
	}
}

/**
* Fixup a couple of angles so that they interpolate the shortest distance between
* each other.
***********************************************************************************/

void UFlippableSpringArmComponent::ModifyRotationBasis(float& fromAngle, float& toAngle) const
{
	float difference = FMathEx::GetSignedDegreesDifference(fromAngle, toAngle);

	// The difference is +/- 180 degrees.

	fromAngle += 360.0f;
	toAngle = fromAngle + difference;
}

/**
* Fixup a couple of rotations so that they interpolate the shortest distance between
* each other.
*
* Rotation interpolation can take you around a complete cycle when the initial
* rotations are not favorable. If we start at -170 for example, and then
* interpolate all the way around to 170, then we have nearly a full rotation when
* perhaps just the 20 degrees would have sufficed. We can determine this by looking
* at the shortest route between the rotations are the start of the transitions and
* then modify the basis of the rotations on each axis so that we follow that route
* innately.
***********************************************************************************/

void UFlippableSpringArmComponent::ModifyRotationBasis(FRotator& fromRotation, FRotator& toRotation)
{
	ModifyRotationBasis(fromRotation.Roll, toRotation.Roll);
	ModifyRotationBasis(fromRotation.Pitch, toRotation.Pitch);
	ModifyRotationBasis(fromRotation.Yaw, toRotation.Yaw);
}

/**
* Make the arm offset in world space from a particular following mode.
***********************************************************************************/

FVector UFlippableSpringArmComponent::MakeArmOffset(FCameraOffset& cameraOffset, FRotator& rotation, EFollowingMode followingMode, bool includeVerticalOffset) const
{
	float verticalScale = (includeVerticalOffset == true) ? 1.0f : 0.0f;
	bool usingOffset = (FollowingModeVectors[(int32)followingMode].IsZero() == false);

	// Now offset camera position back along our orbiting rotation.

	FVector armOffset = FVector(cameraOffset.XOffset, 0.0f, (usingOffset == false) ? cameraOffset.ZOffset * verticalScale : 0.0f);

	// Add in the orbit factor.

	armOffset = FRotator((usingOffset == true) ? 0.0f : OrbitVer, OrbitHor, 0.0f).RotateVector(armOffset);

	// And scale it according to the field of view fraction (which is connected to speed)
	// to make the camera appear to lag the vehicle at speed.

	float scale = (1.0f - (FieldOfViewBias * (FieldOfViewCompensation / 100.0f)));

	armOffset *= scale;

	// Now rotate the arm offset.

	armOffset = rotation.RotateVector(armOffset);

	if (usingOffset == true)
	{
		// Add in the world space offset.

		armOffset += FollowingModeVectors[(int32)followingMode] * cameraOffset.ZOffset * scale * verticalScale;
	}

	return armOffset;
}

/**
* Make the arm root as a point on the vehicle to clip towards, but never past.
***********************************************************************************/

FVector UFlippableSpringArmComponent::MakeArmRoot(const FVector& attachmentRoot, const FVector& armOffset, bool flipped)
{
	// Calculate which is the uppermost side of the vehicle with respect to the camera.

	// attachmentRoot should just be the center of the vehicle, so its location in world space.
	// armOffset is the offset from the attachment root to place the camera at in world space.

	FVector cameraOrigin = attachmentRoot + armOffset;
	ABaseVehicle* vehicle = Cast<ABaseVehicle>(GetAttachmentRootActor());
	const FTransform& vehicleTransform = vehicle->VehicleMesh->GetComponentTransform();

	// If none of the surfaces are uppermost (the camera is between the two of them) then we need to
	// clip to the edges of the sides of the vehicle instead. The arm will certainly clip one of them
	// - we just need to find which one and calculate the intersection point.

	// Create a rectangle that represents the upper surface of the vehicle's bounding box.

	FVector boundingExtent = vehicle->GetBoundingExtent();
	float halfWidth = boundingExtent.Y;
	float halfLength = boundingExtent.X;
	FVector vehicleUp = vehicleTransform.GetUnitAxis(EAxis::Z);
	FVector vehicleDown = vehicleUp * -1.0f;
	int32 surface = (flipped == true) ? -1 : +1;
	float aboveUpper = FPlane::PointPlaneDist(cameraOrigin, vehicleTransform.GetTranslation() + (vehicleUp * boundingExtent.Z), vehicleUp);
	float belowLower = FPlane::PointPlaneDist(cameraOrigin, vehicleTransform.GetTranslation() + (vehicleDown * boundingExtent.Z), vehicleDown);

	if (aboveUpper >= 0.0f)
	{
		// The camera is above the top plane.

		surface = +1;
	}
	else if (belowLower >= 0.0f)
	{
		// The camera is below the bottom plane.

		surface = -1;
	}
	else
	{
		// We're in between the top and bottom planes of the clip volume, so we need to test each of
		// the sides for the closest hit point.

		ArmRootMode = 3;

		FVector vehicleXP = vehicleTransform.GetUnitAxis(EAxis::X);
		FVector vehicleXN = vehicleXP * -1.0f;
		FVector vehicleYP = vehicleTransform.GetUnitAxis(EAxis::Y);
		FVector vehicleYN = vehicleYP * -1.0f;

		boundingExtent += FVector(20.0f, 20.0f, 0.0f);

		halfWidth = boundingExtent.Y;
		halfLength = boundingExtent.X;

		FVector origins[4] =
		{
			vehicleTransform.GetTranslation() + (vehicleXP * boundingExtent.X), vehicleTransform.GetTranslation() + (vehicleXN * boundingExtent.X),
			vehicleTransform.GetTranslation() + (vehicleYP * boundingExtent.Y), vehicleTransform.GetTranslation() + (vehicleYN * boundingExtent.Y)
		};

		FVector normals[4] =
		{
			vehicleXP, vehicleXN, vehicleYP, vehicleYN
		};

		for (int32 i = 0; i < 4; i++)
		{
			float p0 = FPlane::PointPlaneDist(attachmentRoot, origins[i], normals[i]);
			float p1 = FPlane::PointPlaneDist(attachmentRoot + armOffset, origins[i], normals[i]);

			if (FMathEx::UnitSign(p0) != FMathEx::UnitSign(p1))
			{
				// The line crosses this plane, so now find the intersection point.

				FVector planeIntersection = FMath::LinePlaneIntersection(attachmentRoot + armOffset, attachmentRoot, origins[i], normals[i]);

				if (planeIntersection.ContainsNaN() == false)
				{
					FVector localIntersection = vehicleTransform.InverseTransformPosition(planeIntersection);

					if ((FMath::Abs(localIntersection.X) < halfLength + 0.1f) &&
						(FMath::Abs(localIntersection.Y) < halfWidth + 0.1f))
					{
						// The intersection point is on the surface of the bounding box.

						return planeIntersection;
					}
				}
			}
		}

		ensureMsgf(0, TEXT("Didn't find a plane to hit"));

		// We didn't find a plane to hit, which should happen next to never so just return something
		// usable in this rare instance.

		return attachmentRoot + armOffset;
	}

	// Use the "top" surface if +1, or the "bottom" surface if -1.

	FVector surfaceNormal = vehicleTransform.TransformVector(FVector(0.0f, 0.0f, (float)surface));
	FVector surfaceOffset = surfaceNormal * boundingExtent.Z;

	// Calculate the intersection point of the arm offset and the plane.

	FVector planeIntersection = FMath::LinePlaneIntersection(attachmentRoot + armOffset, attachmentRoot, vehicleTransform.GetTranslation() + surfaceOffset, surfaceNormal);

	if (planeIntersection.ContainsNaN() == false)
	{
		FVector localIntersection = vehicleTransform.InverseTransformPosition(planeIntersection);

		if ((FMath::Abs(localIntersection.X) < halfLength + 0.1f) &&
			(FMath::Abs(localIntersection.Y) < halfWidth + 0.1f))
		{
			// The intersection is inside the bounding box so project it out to the edge of the box.

			ArmRootMode = 1;

			return planeIntersection;
		}
	}

	// If no intersection found, calculate the nearest point between the arm offset and each of the
	// planform rectangle's edges. Doing this will try to keep the camera above / below the vehicle
	// if it already was rather than pushing it towards one of the side edges.

	ArmRootMode = 2;

	FVector localOrigin = vehicleTransform.InverseTransformPosition(attachmentRoot);
	FVector localDirection = vehicleTransform.InverseTransformVector(armOffset);

	FMathEx::FRectangle rectangle = { FVector2D(-halfWidth, -halfLength), FVector2D(halfWidth, halfLength) };

	FVector2D from = FVector2D(localOrigin.Y, localOrigin.X);
	FVector2D to = from + FVector2D(localDirection.Y, localDirection.X);

	FMathEx::CohenSutherlandLineClip(from, to, rectangle);

	return vehicleTransform.TransformPosition(FVector(to.Y, to.X, boundingExtent.Z * surface));
}

/**
* Update the rotation offset, used to emphasize drifting.
***********************************************************************************/

void UFlippableSpringArmComponent::UpdateRotationOffset(float deltaSeconds, ABaseVehicle* vehicle, float lagRatio)
{
	float yaw = -vehicle->GetSpringArmYaw() * lagRatio;
	float roll = -vehicle->GetSpringArmRoll() * lagRatio;
	float fraction = FMathEx::GetSmoothingRatio(0.975f, deltaSeconds);

	RotationOffset.Yaw = (RotationOffset.Yaw * fraction) + (yaw * (1.0f - fraction));
	RotationOffset.Roll = (RotationOffset.Roll * fraction) + (roll * (1.0f - fraction));

	RotationOffset.Yaw *= 1.0f - GetCrashedTransitionRatio();
	RotationOffset.Roll *= 1.0f - GetCrashedTransitionRatio();
}

/**
* Registration of the component.
***********************************************************************************/

void UFlippableSpringArmComponent::OnRegister()
{
	Super::OnRegister();

	UpdateDesiredArmProperties(GetComponentTransform(), GetComponentRotation(), false, false, false, 0.0f);
}

/**
* Do the regular update tick.
***********************************************************************************/

void UFlippableSpringArmComponent::TickComponent(float deltaSeconds, enum ELevelTick tickType, FActorComponentTickFunction* thisTickFunction)
{
	Super::TickComponent(deltaSeconds, tickType, thisTickFunction);

	UAdvancedCameraComponent* camera = Cast<UAdvancedCameraComponent>(GetChildComponent(0));
	ABaseVehicle* vehicle = Cast<ABaseVehicle>(GetAttachmentRootActor());

	if (camera != nullptr &&
		vehicle != nullptr &&
		vehicle->IsVehicleDestroyed() == false)
	{
		// Hide the vehicle if we're in cockpit view.

		if (vehicle->IsAIVehicle() == false)
		{
		}
	}

	// As long as the player isn't using the rear or side views then merge the analog looking
	// values into a single yaw angle.

	if (YawActionOverride == false)
	{
		float angle = FMath::RadiansToDegrees(FMath::Atan2(LookingSideways, -LookingForwards));
		float amount = FMath::Sqrt(LookingSideways * LookingSideways + LookingForwards * LookingForwards);

		if (amount < LookingDeadZone)
		{
			amount = 0.0f;
		}

		TargetUserYawAngle = FMath::Lerp(0.0f, angle, FMath::Min(1.0f, amount));
	}

	float fraction = FMathEx::GetSmoothingRatio(0.8f, deltaSeconds);
	float targetYawAngle = TargetUserYawAngle;

	if (CurrentUserYawAngle > 90.0f && targetYawAngle < -90.0f)
	{
		targetYawAngle = 180.0f - (-180.0f - targetYawAngle);
	}
	else if (CurrentUserYawAngle < -90.0f && targetYawAngle > 90.0f)
	{
		targetYawAngle = -180.0f - (180.0f - targetYawAngle);
	}

	CurrentUserYawAngle = (CurrentUserYawAngle * fraction) + (targetYawAngle * (1.0f - fraction));

	if (CurrentUserYawAngle < -180.0f)
	{
		CurrentUserYawAngle = 180.0f - (-CurrentUserYawAngle - 180.0f);
	}
	else if (CurrentUserYawAngle > 180.0f)
	{
		CurrentUserYawAngle = -180.0f - (180.0f - CurrentUserYawAngle);
	}

	SpeedRollTimer = FMath::Max(SpeedRollTimer - deltaSeconds * 2.5f, 0.0f);

	// When the vehicle is close to the vertical Z axis we have to do something special to avoid
	// asymptotes with the rotation. If we don't do this, the camera will tend to roll around wildly
	// when its facing direction closes in on that vertical axis. This solution here is not great,
	// and there is probably a better way of solving this problem.

	if (vehicle != nullptr)
	{
		FTransform transform = vehicle->VehicleMesh->GetComponentTransform();
		FQuat rotation = vehicle->VehicleMesh->GetComponentQuat();
		float pitch = FMath::Abs(rotation.Rotator().Pitch);

		if (pitch < 80.0f)
		{
			float lastPitch = FMath::Abs(LastGoodVehicleRotation.Rotator().Pitch);

			if (lastPitch < pitch)
			{
				float fromRange = pitch - lastPitch;
				float toRange = 90.0f - lastPitch;

				NinetyDegreeVehicleRotation = FQuat::Slerp(LastGoodVehicleRotation, rotation, toRange / fromRange);
			}

			LastGoodVehicleRotation = rotation;
		}
		else
		{
			if (IsBumperView() == false &&
				vehicle->IsCockpitView() == false &&
				vehicle->IsCinematicCameraActive(false) == false)
			{
				float lastPitch = FMath::Abs(LastGoodVehicleRotation.Rotator().Pitch);
				float ratio = FMathEx::GetRatio(pitch, lastPitch, 90.0f);
				FQuat predictedRotation = FQuat::Slerp(LastGoodVehicleRotation, NinetyDegreeVehicleRotation, ratio);

				ratio = FMathEx::GetRatio(pitch, lastPitch, 85.0f);
				rotation = FQuat::Slerp(rotation, predictedRotation, ratio);
				transform = FTransform(rotation, transform.GetTranslation());
			}
		}

		UpdateDesiredArmProperties(transform, rotation.Rotator(), true, true, true, deltaSeconds);
	}

	// Form a transform for new world transform for camera.

	FTransform cameraWorld(CurrentRotation, CurrentLocation);

	// Convert to relative to component.

	FTransform cameraRelative = cameraWorld.GetRelativeTransform(GetComponentTransform());

	// Update socket location/rotation.

	RelativeSocketLocation = cameraRelative.GetLocation();
	RelativeSocketRotation = cameraRelative.GetRotation();

	if (OrbitHor != 0.0f)
	{
		RelativeSocketRotation *= FRotator(0.0f, OrbitHor, 0.0f).Quaternion();
	}

	// Now take into account the angle we want to adjust pitch at by for the target length.

	RelativeSocketRotation *= FRotator(OrbitVer - DownAngle, 0.0f, 0.0f).Quaternion();
}

/**
* Get a transform for the socket the spring arm is exposing.
***********************************************************************************/

FTransform UFlippableSpringArmComponent::GetSocketTransform(FName InSocketName, ERelativeTransformSpace TransformSpace) const
{
	FTransform RelativeTransform(RelativeSocketRotation, RelativeSocketLocation);

	switch (TransformSpace)
	{
	case RTS_World:
		return RelativeTransform * GetComponentTransform();

	case RTS_Actor:
		if (const AActor* Actor = GetOwner())
		{
			FTransform SocketTransform = RelativeTransform * GetComponentTransform();
			return SocketTransform.GetRelativeTransform(Actor->GetTransform());
		}
		break;

	default:
		break;
	}

	return RelativeTransform;
}

/**
* Ease the camera in toward the target.
***********************************************************************************/

void UFlippableSpringArmComponent::CameraIn()
{
	CameraAt(CameraOffsetIndex + 1);

	ABaseVehicle* vehicle = Cast<ABaseVehicle>(GetAttachmentRootActor());
	TArray<int32>& racePositions = UGlobalGameState::GetGlobalGameState(GetWorld())->TransientGameState.RaceCameraPositions;

	if (racePositions.IsValidIndex(vehicle->LocalPlayerIndex) == true)
	{
		racePositions[vehicle->LocalPlayerIndex] = CameraOffsetIndex;
	}
}

/**
* Ease the camera out away from the target.
***********************************************************************************/

void UFlippableSpringArmComponent::CameraOut()
{
	CameraAt(CameraOffsetIndex - 1);

	ABaseVehicle* vehicle = Cast<ABaseVehicle>(GetAttachmentRootActor());
	TArray<int32>& racePositions = UGlobalGameState::GetGlobalGameState(GetWorld())->TransientGameState.RaceCameraPositions;

	if (racePositions.IsValidIndex(vehicle->LocalPlayerIndex) == true)
	{
		racePositions[vehicle->LocalPlayerIndex] = CameraOffsetIndex;
	}
}

/**
* Set the camera to an offset from the target.
***********************************************************************************/

void UFlippableSpringArmComponent::CameraAt(int32 index)
{
	int32 lastCameraOffsetIndex = CameraOffsetIndex;

	CameraOffsetIndex = FMath::Clamp(index, 0, CameraOffsets.Num());

	SetupCameraOffsets(lastCameraOffsetIndex);
}

/**
* Setup the camera offsets for the spring arm.
***********************************************************************************/

void UFlippableSpringArmComponent::SetupCameraOffsets(int32 lastCameraOffsetIndex)
{
	if (IsCockpitView() == true)
	{
		// Cockpit camera.

		CameraOffsetFrom = CameraOffsetTo;
		CameraOffsetTime = 0.0f;
	}
	else
	{
		// Regular camera.

		CameraOffsetFrom.InterpEaseInOut(CameraOffsetFrom, CameraOffsetTo, CameraOffsetTime, 2.0f);
		CameraOffsetTo = CameraOffsets[CameraOffsetIndex];
		CameraOffsetTime = 0.0f;

		if (CameraOffsetTo.LagRatio < KINDA_SMALL_NUMBER)
		{
			if (BodyAttachment == false)
			{
				ABaseVehicle* vehicle = Cast<ABaseVehicle>(GetAttachmentRootActor());

				DetachFromComponent(FDetachmentTransformRules::KeepRelativeTransform);

				GRIP_ATTACH(this, vehicle->VehicleMesh, "RootDummy");

				BodyAttachment = true;
			}

			CameraOffsetFrom = CameraOffsetTo;
		}
		else
		{
			if (BodyAttachment == true)
			{
				ABaseVehicle* vehicle = Cast<ABaseVehicle>(GetAttachmentRootActor());

				DetachFromComponent(FDetachmentTransformRules::KeepRelativeTransform);

				GRIP_ATTACH(this, vehicle->VehicleMesh, NAME_None);

				BodyAttachment = false;

				CameraOffsetFrom = CameraOffsetTo;
			}
		}
	}
}

/**
* Is the owner of this vehicle being watched in any viewport?
***********************************************************************************/

bool UFlippableSpringArmComponent::OwnerIsBeingWatched() const
{
	ABaseVehicle* thisVehicle = Cast<ABaseVehicle>(GetAttachmentRootActor());

	if (thisVehicle->IsHumanPlayer() == true)
	{
		return true;
	}

	APlayGameMode* playGameMode = APlayGameMode::Get(this);

	if (playGameMode != nullptr)
	{
		GRIP_GAME_MODE_LIST_FOR_FROM(GetVehicles(), vehicles, playGameMode);

		for (ABaseVehicle* vehicle : vehicles)
		{
			if ((vehicle->IsHumanPlayer() == true) &&
				(vehicle == thisVehicle))
			{
				return true;
			}
		}
	}

	return false;
}

/**
* Clip the spring arm against other vehicles.
***********************************************************************************/

bool UFlippableSpringArmComponent::ClipAgainstVehicles(const FVector& start, FVector& end) const
{
	bool result = false;
	ABaseVehicle* thisVehicle = Cast<ABaseVehicle>(GetAttachmentRootActor());
	APlayGameMode* playGameMode = APlayGameMode::Get(this);

	if (playGameMode != nullptr)
	{
		GRIP_GAME_MODE_LIST_FROM(GetVehicles(), vehicles, playGameMode);

		// Check each of the vehicles in the game against the spring-arm.

		for (ABaseVehicle* vehicle : vehicles)
		{
			if (vehicle != thisVehicle)
			{
				FBox box = vehicle->CameraClipBox;
				FVector halfVector = (end - start) * 0.5f;
				FVector center = start + halfVector;
				float radius = halfVector.Size();

				// If this vehicle is close enough to us to warrant a clip check then do just that.

				if ((vehicle->GetActorLocation() - center).Size() < box.Max.Size() + radius + 200.0f)
				{
					float hitTime;
					FVector hitNormal;
					FVector hitLocation;
					const FTransform& transform = vehicle->VehicleMesh->GetComponentTransform();
					FVector vehicleStart = transform.InverseTransformPosition(start);
					FVector vehicleEnd = transform.InverseTransformPosition(end);

					if (FMath::LineExtentBoxIntersection(vehicle->CameraClipBox, vehicleStart, vehicleEnd, FVector::ZeroVector, hitLocation, hitNormal, hitTime) == true)
					{
						end = transform.TransformPosition(hitLocation);

						result = true;
					}
				}
			}
		}
	}

	return result;
}

#pragma endregion VehicleSpringArm
