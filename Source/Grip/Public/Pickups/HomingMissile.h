/**
*
* Scorpion homing missile implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Homing missile pickup type, one of the pickups used by vehicles in the game.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "pickupbase.h"
#include "components/staticmeshcomponent.h"
#include "physicsengine/radialforcecomponent.h"
#include "pickups/missilemovementcomponent.h"
#include "effects/lightstreakcomponent.h"
#include "homingmissile.generated.h"

struct FPlayerPickupSlot;

/**
* Boilerplate class for the MissileHostInterface.
***********************************************************************************/

UINTERFACE(MinimalAPI, meta = (CannotImplementInterfaceInBlueprint))
class UMissileHostInterface : public UInterface
{
	GENERATED_UINTERFACE_BODY()
};

/**
* Interface class for the MissileHostInterface.
***********************************************************************************/

class IMissileHostInterface
{
	GENERATED_IINTERFACE_BODY()

public:

	// Use human player audio?
	virtual bool UseHumanPlayerAudio() const = 0;

	// Get the unique index for the vehicle host.
	virtual int32 GetVehicleIndex() const
	{ return -1; }
};

/**
* Legacy class derived from UParticleSystemComponent, no longer extends it.
***********************************************************************************/

UCLASS(ClassGroup = Rendering)
class GRIP_API UGripTrailParticleSystemComponent : public UParticleSystemComponent
{
	GENERATED_BODY()
};

/**
* The homing missile pickup actor.
***********************************************************************************/

UCLASS(Abstract, ClassGroup = Pickups)
class GRIP_API AHomingMissile : public APickupBase
{
	GENERATED_BODY()

public:

	// Construct a homing missile.
	AHomingMissile();

	// The amount of variance of the angle of ejection, for untargeted missiles.
	UPROPERTY(EditAnywhere, Category = Missile, meta = (UIMin = "0", UIMax = "1", ClampMin = "0", ClampMax = "1"))
		float AngleVariance = 0.1f;

	// Rocket duration in seconds.
	UPROPERTY(EditAnywhere, Category = Missile)
		float RocketDuration = 10.0f;

	// The relative force of the explosion against the target vehicle.
	UPROPERTY(EditAnywhere, Category = Missile, meta = (UIMin = "0", UIMax = "100", ClampMin = "0", ClampMax = "100"))
		float VehicleExplosionForce = 1.0f;

	// The hit points associated with this weapon.
	UPROPERTY(EditAnywhere, Category = Missile, meta = (UIMin = "0", UIMax = "250", ClampMin = "0", ClampMax = "250"))
		int32 HitPoints = 40;

	// Should we lose lock with the target when the target is behind the missile?
	UPROPERTY(EditAnywhere, Category = Missile)
		bool LoseLockOnRear = true;

	// The distance for the proximity fuse.
	UPROPERTY(EditAnywhere, Category = Missile)
		float ProximityFuse = 1000.0f;

	// Sound cue for the eject sound.
	UPROPERTY(EditAnywhere, Category = Missile)
		USoundCue* EjectSound = nullptr;

	// Sound cue for the eject sound. Non-Player variant.
	UPROPERTY(EditAnywhere, Category = Missile)
		USoundCue* EjectSoundNonPlayer = nullptr;

	// Sound cue for the ignition sound.
	UPROPERTY(EditAnywhere, Category = Missile)
		USoundCue* IgnitionSound = nullptr;

	// Sound cue for the ignition sound. Non-Player variant.
	UPROPERTY(EditAnywhere, Category = Missile)
		USoundCue* IgnitionSoundNonPlayer = nullptr;

	// Sound cue for the rocket sound.
	UPROPERTY(EditAnywhere, Category = Missile)
		USoundCue* RocketSound = nullptr;

	// Sound cue for the explosion sound.
	UPROPERTY(EditAnywhere, Category = Missile)
		USoundCue* ExplosionSound = nullptr;

	// Sound cue for the explosion sound.
	UPROPERTY(EditAnywhere, Category = Missile)
		USoundCue* ExplosionSoundNonPlayer = nullptr;

	// The particle system to use for airborne the missile explosion.
	UPROPERTY(EditAnywhere, Category = Missile)
		UParticleSystem* ExplosionVisual = nullptr;

	// The radial force for the explosion.
	UPROPERTY(EditAnywhere, Category = Missile)
		URadialForceComponent* ExplosionForce = nullptr;

	// The mesh to use as a missile.
	UPROPERTY(EditAnywhere, Category = Missile)
		UStaticMeshComponent* MissileMesh = nullptr;

	// The particle system to use for rocket trail.
	UPROPERTY(EditAnywhere, Category = Missile)
		UGripTrailParticleSystemComponent* RocketTrail = nullptr;

	// The light streak to use for rocket trail.
	UPROPERTY(EditAnywhere, Category = Missile)
		ULightStreakComponent* RocketLightStreak = nullptr;

	// The light to use for rocket trail.
	UPROPERTY(EditAnywhere, Category = Missile)
		UPointLightComponent* RocketLight = nullptr;

	// The movement used for the missile.
	UPROPERTY(EditAnywhere, Category = Missile)
		UMissileMovementComponent* MissileMovement = nullptr;

	// The target actor that the missile is aiming for right now.
	UPROPERTY(Transient)
		AActor* Target = nullptr;

	// Audio component for the rocket sound.
	UPROPERTY(Transient)
		UAudioComponent* RocketAudio = nullptr;

	friend class ADebugMissileHUD;
};
