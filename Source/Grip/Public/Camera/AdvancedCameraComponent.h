/**
*
* Advanced camera implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* An advanced camera component to give a lot of helper functionality to generalized
* camera work. It has 3 modes of operation, native, custom and mouse control, and
* has transition capabilities to interpolate between each mode when required. Native
* mode has the actor to which it's attached controlling its location and
* orientation. Custom, for when you want complete control like in cinematic cameras.
* And mouse, often used during the game's development but rarely in the field.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "camera/cameracomponent.h"
#include "camera/cinematicsdirector.h"
#include "system/mathhelpers.h"
#include "advancedcameracomponent.generated.h"

/**
* An advanced camera component for adding helper functionality to generalized camera
* work.
***********************************************************************************/

UCLASS(ClassGroup = Camera, meta = (BlueprintSpawnableComponent))
class GRIP_API UAdvancedCameraComponent : public UCameraComponent
{
	GENERATED_BODY()

public:

	// Construct an advanced camera component.
	UAdvancedCameraComponent();

	// Is the location to receive smoothing?
	UPROPERTY(EditAnywhere, Category = AdvancedCameraSettings)
		bool SmoothLocation = false;

	// Controls how quickly camera reaches target location.
	UPROPERTY(EditAnywhere, Category = AdvancedCameraSettings, meta = (EditCondition = "SmoothLocation"))
		float LocationLagRatio = 0.9f;

	// Is the rotation to receive smoothing?
	UPROPERTY(EditAnywhere, Category = AdvancedCameraSettings)
		bool SmoothRotation = false;

	// Controls how quickly camera reaches target yaw angle.
	UPROPERTY(EditAnywhere, Category = AdvancedCameraSettings, meta = (EditCondition = "SmoothRotation"))
		float YawLagRatio = 0.9f;

	// Controls how quickly camera reaches target pitch angle.
	UPROPERTY(EditAnywhere, Category = AdvancedCameraSettings, meta = (EditCondition = "SmoothRotation"))
		float PitchLagRatio = 0.9f;

	// Controls how quickly camera reaches roll angle.
	UPROPERTY(EditAnywhere, Category = AdvancedCameraSettings, meta = (EditCondition = "SmoothRotation"))
		float RollLagRatio = 0.9f;

	// Is the mouse rotation to receive smoothing?
	UPROPERTY(EditAnywhere, Category = AdvancedCameraSettings)
		bool SmoothMouseRotation = true;

	// Is the roll axis locked?
	UPROPERTY(EditAnywhere, Category = AdvancedCameraSettings)
		bool LockRollAxis = false;

	// Returns camera's point of view, more quickly if we don't required post processing information.
	void GetCameraViewNoPostProcessing(float deltaSeconds, FMinimalViewInfo& desiredView)
	{
		float temp = PostProcessBlendWeight;
		PostProcessBlendWeight = 0.0f;
		GetCameraView(deltaSeconds, desiredView);
		PostProcessBlendWeight = temp;
	}
};
