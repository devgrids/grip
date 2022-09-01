/**
*
* Flippable vehicle physics implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Data structures used to manage the physics handling of a vehicle.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "system/timesmoothing.h"
#include "system/mathhelpers.h"

#pragma region MinimalVehicle

/**
* Enumeration for the state of the auto-boost.
***********************************************************************************/

UENUM(BlueprintType)
enum class EAutoBoostState : uint8
{
	Charging,
	Discharging
};

/**
* Data for controlling the propulsion of a vehicle.
***********************************************************************************/

struct FVehiclePropulsion
{
	// Has propulsion been applied since the race start?
	bool HasStarted = false;

	// The applied piston engine throttle of the vehicle between -1 and +1.
	// This is just the throttle position moderated by the brake position (full brake = no thrust).
	float PistonEngineThrottle = 0.0f;

	// The applied jet engine throttle of the vehicle between -1 and +1.
	// This is just the throttle position moderated by the brake position (full brake = no thrust).
	float JetEngineThrottle = 0.0f;

	// The amount of jet engine power currently available to the throttle.
	float CurrentJetEnginePower = 0.0f;

	// The maximum amount of jet engine power available.
	float MaxJetEnginePower = 0.0f;

	// The maximum amount of jet engine power available when airborne.
	float MaxJetEnginePowerAirborne = 0.0f;

	// The current transition time in to / out of idle, 0 being idle and 1 being not.
	float IdleTransitionTime = 0.0f;

	// Transition in to / out of idle direction -1 being in, +1 being out.
	float IdleTransitionDirection = -1.0f;

	// Has the player throttled off since they've been airborne?
	bool ThrottleOffWhileAirborne = false;

	// Boost to the thrust from a turbo-type device.
	float Boost = 0.0f;

	// How much to scale the grip by when using a turbo boost.
	float BoostGripScale = 1.0f;

	// The current state of the auto-boost.
	EAutoBoostState AutoBoostState = EAutoBoostState::Charging;

	// The current volume level to use for boost audio.
	float AutoBoostVolume = 0.0f;

	// The amount of auto-boost available.
	float AutoBoost = 0.0f;

	// The amount to shake the camera by when using auto-boost.
	float AutoBoostShake = 0.0f;

	// How much the raise the front-end by when using a turbo boost.
	float RaiseFrontScale = 0.0f;

	// The time at which the front-end raising was achieved.
	float RaiseFrontAchieved = 0.0f;

	// The thrust strength of the current turbo between 0 and 1.
	float TurboThrottle = 0.0f;

	// The amount of power available to antigravity vehicles, as its cut when they start on the grid and after they've been attacked.
	float AirPower = 0.0f;

	// The amount of time remaining where air power is being cut.
	float AirPowerCut = 0.0f;

	// The last gear.
	int32 LastGear = 0;

	// The current gear position.
	float CurrentGearPosition = 0.0f;

	// The last gear position.
	float LastGearPosition = 0.0f;

	// The amount of time spent in the current gear.
	float GearTime = 0.0f;
};

/**
* Data for calculating physics timing.
***********************************************************************************/

struct FPhysicsTiming
{
	// The amount of time pumped through the physics sub-stepping.
	float TickSum = 0.0f;

	// The amount of time pumped through general Tick.
	float GeneralTickSum = 0.0f;

	// The number of physics sub-step ticks.
	int32 TickCount = -100;

	// The number of general ticks.
	int32 GeneralTickCount = 0;

	// The delta seconds for the last physics sub-step.
	float LastSubstepDeltaSeconds = 0.0f;
};

/**
* Data for the contact state of a vehicle.
***********************************************************************************/

struct FPhysicsContactData
{
	// Is the vehicle currently with all wheels off the ground?
	bool Airborne = false;

	// Was the vehicle with all wheels off the ground on the last frame?
	bool WasAirborne = false;

	// Is the vehicle currently with all wheels on the ground?
	bool Grounded = false;

	// Has the vehicle landed after a respawn?
	bool RespawnLanded = false;

	// The amount of time spent in the current mode (airborne or grounded to some degree)
	float ModeTime = 0.0f;

	// The amount of time spent in the current mode (airborne or grounded to some degree) in the previous contact mode.
	float LastModeTime = 0.0f;

	// How long has the vehicle apparently been falling for?
	float FallingTime = 0.0f;

	// Record of grounded value values.
	FTimedFloatList GroundedList = FTimedFloatList(5, 10);

	// Record of airborne value values.
	FTimedFloatList AirborneList = FTimedFloatList(5, 10);
};

/**
* Data for the velocity and speed of a physics body.
***********************************************************************************/

struct FPhysicsVelocityData
{
	//Set the velocities and related data for the physics state.
	void SetVelocities(const FVector& linearVelocity, const FVector& angularVelocity, const FVector& xdirection);

	// The current speed of the vehicle, in centimeters per second.
	float Speed = 0.0f;

	// The current, directed speed of the vehicle, in centimeters per second. With < 0 meaning reversing and > 0 meaning forward travel.
	// If traveling with some slip with direction vs. velocity at all then this will reduce directed speed, down to 0 when completely sideways on.
	float DirectedSpeed = 0.0f;

	// The current linear velocity of the vehicle in world space, in centimeters per second.
	FVector Velocity = FVector::ZeroVector;

	// The current linear velocity direction of the vehicle in world space.
	FVector VelocityDirection = FVector::ForwardVector;

	// The last linear velocity direction of the vehicle in world space.
	FVector LastVelocityDirection = FVector::ForwardVector;

	// Acceleration in local space, in centimeters per second.
	FVector AccelerationLocalSpace = FVector::ZeroVector;

	// Acceleration in world space, in centimeters per second.
	FVector AccelerationWorldSpace = FVector::ZeroVector;

	// The angular velocity in local space.
	// X = Roll, Y = Pitch, Z = Yaw;
	FVector AngularVelocity = FVector::ZeroVector;
};

/**
* Data for bouncing a vehicle in a controlled manner following a hard landing.
***********************************************************************************/

struct FPhysicsBounce
{
	// A specific force strength to induce bounces on landing independent of the suspension
	// which is really just used to keep the vehicle level.
	float Force = 0.0f;

	// The timer used to coordinate the bounce action.
	float Timer = 0.0f;

	// The stage the bounce action is at.
	int32 Stage = 0;

	// The direction in which to bounce the vehicle.
	FVector Direction = FVector::ForwardVector;
};

/**
* Data for holding a vehicle in a static pose at the start of an event.
***********************************************************************************/

struct FPhysicsStaticHold
{
	// Is the static hold active?
	bool Active = false;

	// The location of the static hold.
	FVector Location = FVector::ZeroVector;

	// The rotation of the static hold.
	FQuat Rotation = FQuat::Identity;
};

/**
* Data for drifting a vehicle in a controlled manner.
***********************************************************************************/

struct FPhysicsDrifting
{
	// Is the vehicle currently purposefully drifting?
	bool Active = false;

	// How long have we been in the current drifting state, drifting or not drifting?
	float Timer = 0.0f;

	// How long have we been not drifting?
	float NonDriftingTimer = 0.0f;

	// The angle that the rear-end is currently drifting at.
	float RearDriftAngle = 0.0f;
};

/**
* Data for the physics state of a vehicle.
***********************************************************************************/

struct FVehiclePhysics
{
	// Record of local yaw change values.
	FTimedFloatList PitchChangeList = FTimedFloatList(10, 25, false, true);

	// Record of velocity direction pitch values.
	// This has a high sampling rate as we want to ensure we have the latest information for use
	// by the physics system and reactions are fast.
	FTimedFloatList VelocityPitchList = FTimedFloatList(5, 200, false);

	// Record of angular velocity pitch values.
	FTimedFloatList AngularPitchList = FTimedFloatList(5, 25);

	// Used for measuring how different a vehicle's direction is compared to its velocity vector.
	// This helps us to determine future path more effectively.
	FTimedVectorList DirectionVsVelocityList = FTimedVectorList(5, 25);

	// Used for detecting and setting up bounces.
	FTimedVectorList VelocityList = FTimedVectorList(5, 25);

	// The timer for velocity pitch mitigation.
	float VelocityPitchMitigationTime = 0.0f;

	// The amount for velocity pitch mitigation.
	float VelocityPitchMitigationAmount = 0.0f;

	// The ratio for velocity pitch mitigation.
	float VelocityPitchMitigationRatio = 0.0f;

	// The force we actually applied on the last sub-step for pitch mitigation.
	float VelocityPitchMitigationForce = 0.0f;

	// The current direction of the vehicle in world space.
	FVector Direction = FVector::ForwardVector;

	// The total strength of gravity for this vehicle.
	float GravityStrength = 0.0f;

	// The distance traveled by the vehicle in meters.
	float DistanceTraveled = 0.0f;

	// The mass of the vehicle.
	float StockMass = 1.0f;

	// The mass of the vehicle.
	float CurrentMass = 1.0f;

	// The mass we use when the vehicle is compressed against a surface as in a centrifugal motion.
	// This is lower than stock mass as we try to ease up on spring compression and harsh collisions.
	float CompressedMass = 1.0f;

	// The transform used in this frame.
	FTransform PhysicsTransform = FTransform::Identity;

	// The transform used in the last frame.
	FTransform LastPhysicsTransform = FTransform::Identity;

	// The time the vehicle last spawned a hit effect from a collision.
	float LastHit = -1.0f;

	// The current speed boost from speed pads.
	float SpeedPadBoost = 0.0f;

	// The max angular velocity.
	float MAV = 1000.0f;

	// Timer for adjusting max angular velocity. 0 for airborne and 1 for not, changing over a 1 second interval.
	float MAVTimer = 0.0f;

	// The location where the vehicle spawned into the world.
	FVector StartLocation = FVector::ZeroVector;

	// The rotation where the vehicle spawned into the world.
	FRotator StartRotation = FRotator::ZeroRotator;

	// The location where the vehicle was destroyed in the world.
	FVector DestroyedLocation = FVector::ZeroVector;

	// The rotation where the vehicle was destroyed in the world.
	FRotator DestroyedRotation = FRotator::ZeroRotator;

	float VehicleTBoned = 0.0f;

	// Where to bias the grip, forward or reverse driving, +1 or -1
	float SteeringBias = 1.0f;

	// Where to bias the grip, forward or reverse driving, +1 or -1, smoothed, used as a basis to compute SteeringBias.
	float SmoothedSteeringBias = 1.0f;

	// Where to set the brakes, forward or reverse driving, +1 or -1.
	float BrakingSteeringBias = 1.0f;

	// The side-slip for antigravity vehicles.
	float AntigravitySideSlip = 0.0f;

	// The lateral grip for antigravity vehicles.
	float AntigravityLateralGrip = 1.0f;

	// An impulse to apply which we build-up during the physics sub-step.
	FVector ApplyImpulse = FVector::ZeroVector;

	// Centralize the grip on landing for a moment.
	bool CentralizeGrip = false;

	// Indicator to reset the last location.
	bool ResetLastLocation = true;

	// The location for the last frame, used to calculate movement between frames.
	FVector LastLocation = FVector::ZeroVector;

	// The location where the vehicle was last recorded as grounded.
	FVector LastGroundedLocation = FVector::ZeroVector;

	// Try to balance the spring forces while landing. 1.0f for maximum balancing, 0 for no balancing.
	float SpringScaleTimer = 0.0f;

	// Timer used for dynamically modifying inertia tensor to help avoid bounce flipping behavior.
	float InertiaTensorScaleTimer = 0.0f;

	// The bounds of the original physics asset.
	FBox BodyBounds;

	// Data for timing.
	FPhysicsTiming Timing;

	// Data for the static hold to arrest the vehicle.
	FPhysicsStaticHold StaticHold;

	// Data for the contact state of the vehicle.
	FPhysicsContactData ContactData;

	// Data for the velocity of the vehicle.
	FPhysicsVelocityData VelocityData;

	// Data for drifting the vehicle.
	FPhysicsDrifting Drifting;

	// Data for controllably bouncing the vehicle on heavy landing.
	FPhysicsBounce Bounce;
};

#pragma endregion MinimalVehicle
