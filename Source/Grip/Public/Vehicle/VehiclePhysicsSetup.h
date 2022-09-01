/**
*
* The vehicle physics characteristics.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Data structures used to setup the physics for vehicle handling in the Editor.
*
***********************************************************************************/

#pragma once

#include "vehiclephysicssetup.generated.h"

#pragma region MinimalVehicle

/**
* An enumeration to describe where on a vehicle a wheel is placed.
***********************************************************************************/

UENUM(BlueprintType)
enum class EWheelPlacement : uint8
{
	Rear,
	Front,
	Center
};

/**
* Which kind of tire friction model should be used?
***********************************************************************************/

UENUM(BlueprintType)
enum class ETireFrictionModel : uint8
{
	Arcade
};

/**
* Which kind of wheel drive / brake model should be used?
***********************************************************************************/

UENUM(BlueprintType)
enum class EWheelDriveModel : uint8
{
	FrontWheel,
	RearWheel,
	AllWheel
};

/**
* A tire friction model, governing how friction is calculated. This also works
* for antigravity vehicles too, as the concept is the same despite being
* visually quite different - grip is grip, whatever the packaging.
***********************************************************************************/

UCLASS(ClassGroup = Vehicle)
class UTireFrictionModel : public UDataAsset
{
	GENERATED_BODY()

public:

	UTireFrictionModel();

	// Which friction model to use.
	UPROPERTY(EditAnywhere, Category = General)
		ETireFrictionModel Model = ETireFrictionModel::Arcade;

	// Which braking assignment model to use.
	UPROPERTY(EditAnywhere, Category = General)
		EWheelDriveModel BrakeAssignment = EWheelDriveModel::RearWheel;

	// The lateral grip of the tires, or cornering stiffness, at different velocities for the vehicle.
	// This can be used to specifically tailor the amount of grip, and therefore control handling precisely, all along the speed range.
	UPROPERTY(EditAnywhere, Category = General)
		FRuntimeFloatCurve LateralGripVsSpeed;

	// How much of the base lateral grip is available versus the slip angle of the tire (degrees) when compared to the vehicle's velocity direction.
	UPROPERTY(EditAnywhere, Category = General)
		FRuntimeFloatCurve LateralGripVsSlip;

	// How much to scale the regular lateral grip by for the rear wheels. We need values > 1 in order to stop the rear-end from slipping out at every opportunity.
	// We increase this with speed so that we get a bit of rear-end slip at low-speed but keep you on the straight and narrow at high-speed.
	UPROPERTY(EditAnywhere, Category = General)
		FRuntimeFloatCurve RearLateralGripVsSpeed;

	// The grip of the tires, at relative spring compression.
	// This is normally part of dynamic loading but doing this properly resulted in unpredictable behavior - using a simplified curve here increased that predictability.
	UPROPERTY(EditAnywhere, Category = General)
		FRuntimeFloatCurve GripVsSuspensionCompression;

	// The grip of the antigravity vehicles, at relative spring compression.
	UPROPERTY(EditAnywhere, Category = General)
		FRuntimeFloatCurve GripVsAntigravityCompression;

	// How much of the base longitudinal grip is available versus the slip of the tire (percent) on the driving surface.
	UPROPERTY(EditAnywhere, Category = General)
		FRuntimeFloatCurve LongitudinalGripVsSlip;

	// How much the scale the input slip by when referencing Lateral Grip Vs Slip.
	UPROPERTY(EditAnywhere, Category = General)
		float LateralGripVsSlipScale = 1.0f;

	// The longitudinal grip value (braking grip) for the tire, constant over the entire speed range.
	UPROPERTY(EditAnywhere, Category = General)
		float LongitudinalGripCoefficient = 20.0f;

	// How much speed in meters per second to lose at full braking.
	// This is just for braking wheels from a standing start.
	UPROPERTY(EditAnywhere, Category = General)
		float BrakingDeceleration = 15.0f;

	// How much speed in meters per second to add at full throttle.
	// This is just for spinning wheels from a standing start.
	UPROPERTY(EditAnywhere, Category = General)
		float FlooredAcceleration;

	// How much to kick the back end out when drifting.
	UPROPERTY(EditAnywhere, Category = General, meta = (UIMin = "0.0", UIMax = "25.0", ClampMin = "0.0", ClampMax = "25.0"))
		float RearEndDriftAngle = 10.0f;

	// Speed boost to apply when explicitly drifting to help keep the power-slide going.
	// This is a hack to keep the car moving when you're drifting (otherwise you quickly slow up).
	UPROPERTY(EditAnywhere, Category = Hacks, meta = (UIMin = "0.0", UIMax = "2.0", ClampMin = "0.0", ClampMax = "2.0"))
		float SpeedBoostWhenDrifting = 0.2f;

	// Grip boost to apply when explicitly drifting.
	UPROPERTY(EditAnywhere, Category = Hacks, meta = (UIMin = "0.0", UIMax = "2.0", ClampMin = "0.0", ClampMax = "2.0"))
		float GripBoostWhenDrifting = 0.2f;
};

/**
* The vehicle engine or propulsion model, governing how a vehicle is propelled.
***********************************************************************************/

UCLASS(ClassGroup = Vehicle)
class UVehicleEngineModel : public UDataAsset
{
	GENERATED_BODY()

public:

	UVehicleEngineModel();

	// The amount of power the jet engine develops to provide jet thrust.
	UPROPERTY(EditAnywhere, Category = JetEngine)
		float JetEnginePower = 275000.0f;

	// The amount of power the jet engine develops to provide jet thrust when airborne.
	// We know this should be the same whether airborne or not, but it often doesn't "feel" right.
	UPROPERTY(EditAnywhere, Category = JetEngine)
		float JetEnginePowerAirborne = JetEnginePower * 0.5f;

	// Which wheel drive model to use, front, rear or all wheel drive.
	UPROPERTY(EditAnywhere, Category = PistonEngine)
		EWheelDriveModel WheelDriveModel = EWheelDriveModel::AllWheel;

	// The engine braking coefficient of the vehicle, or how much to slow the vehicle by because no throttle is being applied and the piston engine is naturally slowing the wheels.
	UPROPERTY(EditAnywhere, Category = PistonEngine)
		float EngineBrakingCoefficient = 0.01f;

	// The gear ratio for reverse gear.
	UPROPERTY(EditAnywhere, Category = PistonEngine)
		float ReverseGearRatio = 2.9f;

	// The range of any particular gear, in KPH.
	UPROPERTY(EditAnywhere, Category = GearSimulation)
		float GearSpeedRange = 110.0f;

	// The amount of power available in the lowest gear, from 0.0 to 1.0.
	// By the time you get to 4th gear all the power is then available.
	UPROPERTY(EditAnywhere, Category = GearSimulation)
		float LowGearPowerRatio = 0.7f;

	// The power ratios for each of the gears.
	UPROPERTY(EditAnywhere, Category = GearSimulation)
		TArray<float> GearPowerRatios;

	// The amount of thrust to apply from a standing start when the race begins.
	// Increasing this will shorten the time spent wheel-spinning.
	// This is a hack until we have a physically drive-chain on a piston-engine vehicle.
	UPROPERTY(EditAnywhere, Category = Hacks)
		float StartingThrustRatio = 0.4f;

	// The amount of thrust to apply from a standing start after the race begins.
	// Increasing this will shorten the time spent wheel-spinning.
	// This is a hack until we have a physically drive-chain on a piston-engine vehicle.
	UPROPERTY(EditAnywhere, Category = Hacks)
		float RestartingThrustRatio = 0.6f;

	// The real, rolling RPM the wheels must reach before attaining real traction from a standing start.
	// This also determines how much the wheels spin visually until that speed is attained.
	// This is a hack until we have a physically drive-chain on a piston-engine vehicle.
	UPROPERTY(EditAnywhere, Category = Hacks)
		float StartingWheelSpinRPM = 400.0f;
};

/**
* The steering model, governing how a vehicle is steered.
***********************************************************************************/

UCLASS(ClassGroup = Vehicle)
class USteeringModel : public UDataAsset
{
	GENERATED_BODY()

public:

	USteeringModel();

	// The maximum angle of steering for the front wheels in degrees.
	UPROPERTY(EditAnywhere, Category = Wheels)
		float FrontWheelsMaxSteeringAngle = 20.0f;

	// The maximum angle of steering for the back wheels in degrees.
	UPROPERTY(EditAnywhere, Category = Wheels)
		float BackWheelsMaxSteeringAngle = 10.0f;

	// The amount of visual front under-steer to show the player (doesn't affect handling).
	UPROPERTY(EditAnywhere, Category = Wheels, meta = (UIMin = "0.0", UIMax = "1.0", ClampMin = "0.0", ClampMax = "1.0"))
		float FrontVisualUnderSteerRatio = 0.75f;

	// The amount of visual back under-steer to show the player (doesn't affect handling).
	UPROPERTY(EditAnywhere, Category = Wheels, meta = (UIMin = "0.0", UIMax = "1.0", ClampMin = "0.0", ClampMax = "1.0"))
		float BackVisualUnderSteerRatio = 0.25f;

	// How much the steering angle is reduced by with increasing speed.
	UPROPERTY(EditAnywhere, Category = Wheels)
		FRuntimeFloatCurve FrontSteeringVsSpeed;

	// How much the steering angle is reduced by with increasing speed.
	UPROPERTY(EditAnywhere, Category = Wheels)
		FRuntimeFloatCurve BackSteeringVsSpeed;
};

#pragma endregion MinimalVehicle
