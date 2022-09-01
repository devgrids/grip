/**
*
* Raptor Gatling gun implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Gatling gun pickup type, one of the pickups used by vehicles in the game.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "effects/lightstreakcomponent.h"
#include "pickupbase.h"
#include "gatlinggun.generated.h"

struct FPlayerPickupSlot;

/**
* Boilerplate class for the GunHostInterface.
***********************************************************************************/

UINTERFACE(MinimalAPI, meta = (CannotImplementInterfaceInBlueprint))
class UGunHostInterface : public UInterface
{
	GENERATED_UINTERFACE_BODY()
};

/**
* Interface class for the GunHostInterface.
***********************************************************************************/

class IGunHostInterface
{
	GENERATED_IINTERFACE_BODY()

public:

	// Use human player audio?
	virtual bool UseHumanPlayerAudio() const = 0;
};

/**
* Data for linking a gun to a vehicle.
***********************************************************************************/

UCLASS(ClassGroup = Pickups)
class UVehicleGun : public UDataAsset
{
	GENERATED_BODY()

public:

	// The particle system to use for the muzzle flash.
	UPROPERTY(EditAnywhere, Category = Gun)
		UParticleSystem* MuzzleFlashEffect = nullptr;

	// The particle system to use for the shell ejection.
	UPROPERTY(EditAnywhere, Category = Gun)
		UParticleSystem* ShellEjectEffect = nullptr;

	// The sound to use for the gun.
	UPROPERTY(EditAnywhere, Category = Gun)
		USoundCue* RoundSound = nullptr;

	// The sound to use for the gun.
	UPROPERTY(EditAnywhere, Category = Gun)
		USoundCue* RoundSoundNonPlayer = nullptr;
};

/**
* The Gatling gun pickup actor.
***********************************************************************************/

UCLASS(Abstract, ClassGroup = Pickups)
class GRIP_API AGatlingGun : public APickupBase
{
	GENERATED_BODY()

public:

	// Construct a gun.
	AGatlingGun();

	// The wind up time in seconds.
	UPROPERTY(EditAnywhere, Category = Gun, meta = (UIMin = "0.0", UIMax = "60.0", ClampMin = "0.0", ClampMax = "60.0"))
		float WindUpTime = 2.0f;

	// The wind up time in seconds.
	UPROPERTY(EditAnywhere, Category = Gun, meta = (UIMin = "0.0", UIMax = "60.0", ClampMin = "0.0", ClampMax = "60.0"))
		float WindDownTime = 2.0f;

	// The length of time to fire the gun for, in seconds.
	UPROPERTY(EditAnywhere, Category = Gun, meta = (UIMin = "1.0", UIMax = "10.0", ClampMin = "1.0", ClampMax = "10.0"))
		float Duration = 5.0f;

	// The firing delay time in seconds.
	UPROPERTY(EditAnywhere, Category = Gun, meta = (UIMin = "0.0", UIMax = "60.0", ClampMin = "0.0", ClampMax = "60.0"))
		float FiringDelay = 2.0f;

	// The maximum fire rate of the gun in rounds per second.
	UPROPERTY(EditAnywhere, Category = Gun, meta = (UIMin = "0.0", UIMax = "30.0", ClampMin = "0.0", ClampMax = "30.0"))
		float FireRate = 30.0f;

	// The impact force of the round between 0 and 1.
	UPROPERTY(EditAnywhere, Category = Gun, meta = (UIMin = "0.0", UIMax = "1.0", ClampMin = "0.0", ClampMax = "1.0"))
		float RoundForce = 0.25f;

	// The hit points associated with each round from this weapon.
	UPROPERTY(EditAnywhere, Category = Gun, meta = (UIMin = "0", UIMax = "100", ClampMin = "0", ClampMax = "100"))
		int32 HitPoints = 5;

	// The amount of auto-aiming to apply between 0 and 1.
	UPROPERTY(EditAnywhere, Category = Gun, meta = (UIMin = "0.0", UIMax = "1.0", ClampMin = "0.0", ClampMax = "1.0"))
		float AutoAiming = 0.5f;

	// Alternate the barrels when firing?
	UPROPERTY(EditAnywhere, Category = Gun)
		bool AlternateBarrels = false;

	// Sound cue for the barrel spin sound.
	UPROPERTY(EditAnywhere, Category = Gun)
		USoundCue* BarrelSpinSound = nullptr;

	// Sound cue for the barrel spin sound. Non-Player variant.
	UPROPERTY(EditAnywhere, Category = Gun)
		USoundCue* BarrelSpinSoundNonPlayer = nullptr;

	// Perform some blueprint code when a bullet round hits a surface.
	UFUNCTION(BlueprintImplementableEvent, Category = Gun)
		void BulletHitAnimation(UPrimitiveComponent* component, const TArray<UParticleSystem*>& particleSystems, const TArray<FVector>& hitLocations, USoundCue* soundEffect, const FVector& location, const FRotator& rotation, EGameSurface surface, const FVector& colour, bool charged);

private:

	// Audio component for the barrel spin sound.
	UPROPERTY(Transient)
		UAudioComponent* BarrelSpinAudio = nullptr;
};
