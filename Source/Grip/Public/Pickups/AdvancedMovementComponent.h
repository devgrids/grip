/**
*
* Advanced movement implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* An advanced movement component to give a lot of helper functionality to generalized
* movement work. Mostly, this will center on terrain avoidance.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "gameframework/movementcomponent.h"
#include "system/mathhelpers.h"
#include "advancedmovementcomponent.generated.h"

/**
* Component for controlling advanced movement of other components.
***********************************************************************************/

UCLASS(Abstract)
class GRIP_API UAdvancedMovementComponent : public UMovementComponent
{
	GENERATED_BODY()

public:

	// Construct an advanced movement component.
	UAdvancedMovementComponent();

	// How to smooth direction changes.
	UPROPERTY(EditAnywhere, Category = AdvancedMovement, meta = (UIMin = "0.5", UIMax = "1.0", ClampMin = "0.5", ClampMax = "1.0"))
		float DirectionSmoothingRatio = 0.85f;

	// How to smooth direction changes when avoiding terrain.
	UPROPERTY(EditAnywhere, Category = AdvancedMovement, meta = (UIMin = "0.5", UIMax = "1.0", ClampMin = "0.5", ClampMax = "1.0"))
		float AvoidanceSmoothingRatio = 0.85f;

	UPROPERTY(EditAnywhere, Category = AdvancedMovement)
		bool TerrainHugging = false;

	// How swiftly to get down onto the terrain when above it.
	UPROPERTY(EditAnywhere, Category = AdvancedMovement, meta = (EditCondition = "TerrainHugging"))
		float TerrainHuggingSpeed = 1.0f;

	// The maximum rate at which we try to hug terrain when above it.
	UPROPERTY(EditAnywhere, Category = AdvancedMovement, meta = (EditCondition = "TerrainHugging"))
		float TerrainHuggingMaxSpeed = 3000.0f;

	// Max time delta for each discrete simulation step.
	UPROPERTY(EditAnywhere, Category = AdvancedMovement, meta = (ClampMin = "0.01", ClampMax = "0.1", UIMin = "0.01", UIMax = "0.1"))
		float MaxSimulationTimeStep = 0.05f;

	// Max number of iterations used for each discrete simulation step.
	UPROPERTY(EditAnywhere, Category = AdvancedMovement, meta = (ClampMin = "1", ClampMax = "25", UIMin = "1", UIMax = "25"))
		int32 MaxSimulationIterations = 8;
};
