/**
*
* Vehicle race camera implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* A specialist camera for racing vehicles, which contains a lot gizmos for
* enhancing the sensation of speed and works in conjunction with a specialist
* post processing material.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "camera/advancedcameracomponent.h"
#include "racecameracomponent.generated.h"

class ABaseVehicle;

/**
* A race camera component for adding vehicle-specific enhancements.
***********************************************************************************/

UCLASS(ClassGroup = Camera, hideCategories = Transform, meta = (BlueprintSpawnableComponent))
class GRIP_API URaceCameraComponent : public UAdvancedCameraComponent
{
	GENERATED_BODY()

public:

	// Construct a race camera component.
	URaceCameraComponent();

	// The amount of field of view to use in degrees, vs speed in kilometers per hour.
	UPROPERTY(EditAnywhere, Category = RaceCameraSettings)
		FRuntimeFloatCurve FieldOfViewVsSpeed;

	// The amount of radial speed blur to apply, vs speed in kilometers per hour.
	UPROPERTY(EditAnywhere, Category = RaceCameraSettings)
		FRuntimeFloatCurve RadialSpeedBlurVsSpeed;

	// The amount of fiery ionization to apply, vs speed in kilometers per hour.
	UPROPERTY(EditAnywhere, Category = RaceCameraSettings)
		FRuntimeFloatCurve IonisationVsSpeed;

	// The amount of speed streaking to apply, vs speed in kilometers per hour.
	UPROPERTY(EditAnywhere, Category = RaceCameraSettings)
		FRuntimeFloatCurve StreaksVsSpeed;

	// How much to scale film noise by, affecting its granularity.
	UPROPERTY(EditAnywhere, Category = RaceCameraSettings, meta = (UIMin = "100", UIMax = "10000", ClampMin = "100", ClampMax = "10000"))
		float FilmNoiseScale = 666.0f;

	// The amount of film noise to apply.
	UPROPERTY(EditAnywhere, Category = RaceCameraSettings, meta = (UIMin = "0", UIMax = "1", ClampMin = "0", ClampMax = "1"))
		float FilmNoiseAmount = 0.0f;

private:

	// The cheap, minimal material to use for the race camera.
	UPROPERTY(Transient)
		UMaterialInstanceDynamic* CheapCameraMaterial = nullptr;

	// The expensive, maximal material to use for the race camera.
	UPROPERTY(Transient)
		UMaterialInstanceDynamic* ExpensiveCameraMaterial = nullptr;
};
