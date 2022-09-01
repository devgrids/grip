/**
*
* Missile movement implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* We move the homing missile around using velocity changes and sub-stepping to
* ensure that we maintain a smooth movement arc. The UMissileMovementComponent
* does this work, inheriting from UAdvancedMovementComponent for some of that
* functionality.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "advancedmovementcomponent.h"
#include "missilemovementcomponent.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(GripLogMissile, Warning, All);

/**
* Component for controlling the movement of a homing missile.
***********************************************************************************/

UCLASS()
class GRIP_API UMissileMovementComponent : public UAdvancedMovementComponent
{
	GENERATED_BODY()

public:

	// Construct a missile movement component.
	UMissileMovementComponent();

	// The maximum turn rate of the missile at start speed in degrees per second.
	// Remember this will be limited by Direction Smoothing Ratio.
	UPROPERTY(EditAnywhere, Category = MissileMovement, meta = (UIMin = "0.0", UIMax = "250.0", ClampMin = "0.0", ClampMax = "2500.0"))
		float StartSpeedTurnRate = 250.0f;

	// The maximum turn rate of the missile at top speed in degrees per second.
	// Remember this will be limited by Direction Smoothing Ratio.
	UPROPERTY(EditAnywhere, Category = MissileMovement, meta = (UIMin = "0.0", UIMax = "250.0", ClampMin = "0.0", ClampMax = "2500.0"))
		float TopSpeedTurnRate = 180.0f;

	// Acceleration time in seconds, or 0 for no limit.
	UPROPERTY(EditAnywhere, Category = MissileMovement)
		float AccelerationTime = 3.0f;

	// The magnitude of our acceleration towards the homing target.
	UPROPERTY(EditAnywhere, Category = MissileMovement)
		float HomingAccelerationMagnitude = 0.0f;

	// The maximum speed of the missile in KPH, or 0 for no limit.
	UPROPERTY(EditAnywhere, Category = MissileMovement)
		float MaximumSpeed = 0.0f;

	// The current target we are homing towards. Can only be set at runtime (when projectile is spawned or updating).
	UPROPERTY(Transient)
		USceneComponent* HomingTargetComponent = nullptr;

	friend class ADebugMissileHUD;
};
