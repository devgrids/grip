/**
*
* Painkiller shield implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Shield pickup type, one of the pickups used by vehicles in the game.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "pickupbase.h"
#include "shield.generated.h"

struct FPlayerPickupSlot;

/**
* Data for linking a shield to a vehicle.
***********************************************************************************/

UCLASS(ClassGroup = Pickups)
class UVehicleShield : public UDataAsset
{
	GENERATED_BODY()

public:

	// The offset to use for rendering the front shield.
	UPROPERTY(EditAnywhere, Category = Shield)
		FVector FrontOffset = FVector::ZeroVector;

	// The rotation to use for rendering the front shield.
	UPROPERTY(EditAnywhere, Category = Shield)
		FRotator FrontRotation = FRotator::ZeroRotator;

	// The offset to use for rendering the rear shield.
	UPROPERTY(EditAnywhere, Category = Shield)
		FVector RearOffset = FVector::ZeroVector;

	// The rotation to use for rendering the rear shield.
	UPROPERTY(EditAnywhere, Category = Shield)
		FRotator RearRotation = FRotator::ZeroRotator;

	// The particle system to use for the active state.
	UPROPERTY(EditAnywhere, Category = Shield)
		UParticleSystem* ActiveEffectFront = nullptr;

	// The particle system to use for the destroyed state.
	UPROPERTY(EditAnywhere, Category = Shield)
		UParticleSystem* DestroyedEffectFront = nullptr;

	// The particle system to use for the active state.
	UPROPERTY(EditAnywhere, Category = Shield)
		UParticleSystem* ActiveEffectRear = nullptr;

	// The particle system to use for the destroyed state.
	UPROPERTY(EditAnywhere, Category = Shield)
		UParticleSystem* DestroyedEffectRear = nullptr;

	// The particle system to use for the shield hit effect
	UPROPERTY(EditAnywhere, Category = Shield)
		UParticleSystem* HitEffect = nullptr;

	// The particle system to use for the shield hit effect
	UPROPERTY(EditAnywhere, Category = Shield)
		UParticleSystem* HitPointEffect = nullptr;

	// The sound to use for the shield hit effects
	UPROPERTY(EditAnywhere, Category = Shield)
		USoundCue* HitSound = nullptr;

	// Sound cue for the activate sound.
	UPROPERTY(EditAnywhere, Category = Shield)
		USoundCue* ActivateSound = nullptr;

	// Sound cue for the active sound.
	UPROPERTY(EditAnywhere, Category = Shield)
		USoundCue* ActiveSound = nullptr;

	// Sound cue for the destroyed sound.
	UPROPERTY(EditAnywhere, Category = Shield)
		USoundCue* DestroyedSound = nullptr;
};

/**
* The shield pickup actor.
***********************************************************************************/

UCLASS(Abstract, ClassGroup = Pickups)
class GRIP_API AShield : public APickupBase
{
	GENERATED_BODY()

public:

	// Construct a shield.
	AShield();

	// Is the shield rear-end only?
	UPROPERTY(EditAnywhere, Category = Shield)
		bool RearOnly = true;

	// The hit points associated with the shield.
	UPROPERTY(EditAnywhere, Category = Shield, meta = (UIMin = "0", UIMax = "250", ClampMin = "0", ClampMax = "250"))
		int32 HitPoints = 40;

	// The duration associated with the shield.
	UPROPERTY(EditAnywhere, Category = Shield, meta = (UIMin = "5", UIMax = "30", ClampMin = "5", ClampMax = "30"))
		float Duration = 15.0f;

	// Sound cue for a charged impact.
	UPROPERTY(EditAnywhere, Category = Shield)
		USoundCue* ChargedImpact = nullptr;

	// Looped audio for then the shield is active.
	UPROPERTY(Transient)
		UAudioComponent* ActiveAudio = nullptr;

	// The particle system to use for the active state.
	UPROPERTY(Transient)
		UParticleSystemComponent* ActiveEffectFront = nullptr;

	// The particle system to use for the active state.
	UPROPERTY(Transient)
		UParticleSystemComponent* ActiveEffectRear = nullptr;

	// The particle system to use for the destroyed state.
	UPROPERTY(Transient)
		UParticleSystemComponent* DestroyedEffectFront = nullptr;

	// The particle system to use for the destroyed state.
	UPROPERTY(Transient)
		UParticleSystemComponent* DestroyedEffectRear = nullptr;
};
