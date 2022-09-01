/**
*
* Camera point components.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Camera points attached to vehicles normally, so that we can get a good view of
* the action when the cinematic camera is active.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "camerapointcomponent.generated.h"

/**
* Camera point component for showing a particular view of a vehicle.
***********************************************************************************/

UCLASS(ClassGroup = Camera, meta = (BlueprintSpawnableComponent))
class GRIP_API UCameraPointComponent : public USceneComponent
{
	GENERATED_BODY()

public:

	// Construct a camera point component.
	UCameraPointComponent();

	// Inherit the speed-adjusted field of view from the owning actor?
	UPROPERTY(EditAnywhere, Category = CameraPoint)
		bool InheritSpeedFieldOfView = false;

	// Inherit and render the native effects (e.g. speed blur) from the owning actor?
	UPROPERTY(EditAnywhere, Category = CameraPoint)
		bool InheritNativeEffects = false;

	// The fixed field of view.
	UPROPERTY(EditAnywhere, Category = CameraPoint)
		float FieldOfView = 90.0f;

	// The maximum number of seconds this camera should be used for.
	UPROPERTY(EditAnywhere, Category = CameraPoint)
		float MaximumViewSeconds = 3.0f;

	// Does this camera look okay when the vehicle is experiencing high angular velocities?
	UPROPERTY(EditAnywhere, Category = CameraPoint)
		bool HighAngularVelocities = true;

	// Invert (flip) the Z component when switching to this camera if the owning flippable vehicle is flipped? (if attached to a flippable vehicle)
	UPROPERTY(EditAnywhere, Category = CameraPoint)
		bool InvertWithVehicle = true;

	// Clip the camera when intersecting the environment?
	UPROPERTY(EditAnywhere, Category = CameraPoint)
		bool ClipLocation = true;

	// Clip the camera vertically only when intersecting the environment?
	UPROPERTY(EditAnywhere, Category = CameraPoint, meta = (EditCondition = "ClipLocation"))
		bool ClipVertically = false;

	// Clip the camera horizontally only when intersecting the environment?
	UPROPERTY(EditAnywhere, Category = CameraPoint, meta = (EditCondition = "ClipLocation"))
		bool ClipHorizontally = false;

	// Clip the camera longitudinally only when intersecting the environment?
	UPROPERTY(EditAnywhere, Category = CameraPoint, meta = (EditCondition = "ClipLocation"))
		bool ClipLongitudinally = false;
};
