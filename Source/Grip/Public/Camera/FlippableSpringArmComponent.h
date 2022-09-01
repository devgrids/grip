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
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "system/mathhelpers.h"
#include "flippablespringarmcomponent.generated.h"

/**
* Structure describing the camera offset from its target.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FCameraOffset
{
	GENERATED_USTRUCT_BODY()

public:

	FCameraOffset() = default;

	FCameraOffset(float xoffset, float zoffset, float minDownAngle, float maxDownAngle, float lagRatio)
		: XOffset(xoffset)
		, ZOffset(zoffset)
		, MinDownAngle(minDownAngle)
		, MaxDownAngle(maxDownAngle)
		, LagRatio(lagRatio)
	{ }

	// Interpolate the spring arm between offsets.
	void InterpEaseInOut(const FCameraOffset& from, const FCameraOffset& to, float time, float power)
	{
		XOffset = FMath::InterpEaseInOut(from.XOffset, to.XOffset, time, power);
		ZOffset = FMath::InterpEaseInOut(from.ZOffset, to.ZOffset, time, power);
		MinDownAngle = FMath::InterpEaseInOut(from.MinDownAngle, to.MinDownAngle, time, power);
		MaxDownAngle = FMath::InterpEaseInOut(from.MaxDownAngle, to.MaxDownAngle, time, power);
		LagRatio = FMath::InterpEaseInOut(from.LagRatio, to.LagRatio, time, power);
	}

	// How far in front of the target the camera should be positioned.
	UPROPERTY(EditAnywhere, Category = Offset)
		float XOffset = -500.0f;

	// How far above the target the camera should be positioned.
	UPROPERTY(EditAnywhere, Category = Offset)
		float ZOffset = 200.0f;

	// How much to angle the camera down towards the vehicle in degrees, when moving at high speed.
	UPROPERTY(EditAnywhere, Category = Offset)
		float MinDownAngle = 3.0f;

	// How much to angle the camera down towards the vehicle in degrees, when static.
	UPROPERTY(EditAnywhere, Category = Offset)
		float MaxDownAngle = 5.0f;

	// How much lag should be applied at this offset.
	UPROPERTY(EditAnywhere, Category = Offset)
		float LagRatio = 1.0f;
};

/**
* Class for a spring arm component suitable for a flippable vehicle.
***********************************************************************************/

UCLASS(ClassGroup = Camera, HideCategories = (Transform), meta = (BlueprintSpawnableComponent))
class GRIP_API UFlippableSpringArmComponent : public USceneComponent
{
	GENERATED_BODY()

public:

	// Construct a flippable spring arm component.
	UFlippableSpringArmComponent();

	// Natural length of the spring arm when there are no collisions.
	UPROPERTY(EditAnywhere, Category = Arm)
		TArray<FCameraOffset> CameraOffsets;

	// How big should the query probe sphere be (in unreal units).
	UPROPERTY(EditAnywhere, Category = Arm)
		float ProbeSize = 10.0f;

	// What yaw extension to use?
	UPROPERTY(EditAnywhere, Category = Arm)
		float DriftYawExtension = -1.0f;

	// Controls how quickly camera reaches target pitch angle when the vehicle is on the ground.
	UPROPERTY(EditAnywhere, Category = Lag)
		float CameraPitchLagRatio = 0.98f;

	// Controls how quickly camera reaches target yaw angle when the vehicle is on the ground.
	UPROPERTY(EditAnywhere, Category = Lag)
		float CameraYawLagRatio = 0.90f;

	// Controls how quickly camera reaches target roll angle when the vehicle is on the ground.
	UPROPERTY(EditAnywhere, Category = Lag)
		float CameraRollLagRatio = 0.95f;

	// The amount of forward adjustment to make to compensate for increasing FOV.
	UPROPERTY(EditAnywhere, Category = Camera)
		float FieldOfViewCompensation = 0.8f;

	// Amplitude to shake the camera with high speed.
	UPROPERTY(EditAnywhere, Category = Camera)
		float SpeedShakeAmplitude = 1.0f;

	// Frequency to shake the camera with high speed.
	UPROPERTY(EditAnywhere, Category = Camera)
		float SpeedShakeFrequency = 1.0f;

	// The name of the socket at the end of the spring arm (looking back towards the spring arm origin)
	static const FName SocketName;

private:

	friend class ADebugRaceCameraHUD;
	friend class ADebugVehicleHUD;
};
