/**
*
* Pickup pad implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Pickup pads for vehicles to collect pickups from.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "gameframework/actor.h"
#include "components/staticmeshcomponent.h"
#include "system/commontypes.h"
#include "system/attractable.h"
#include "system/positionable.h"
#include "pickup.generated.h"

class UPursuitSplineComponent;

/**
* Pickup type enumeration, for assigning types to pickup pads.
***********************************************************************************/

UENUM(BlueprintType)
enum class EPickupClass : uint8
{
	// Regular pickup
	Pickup,

	// Collectible pickup for Carkour mode
	Collectible,

	// Health pickup for Arena mode
	Health,

	// Double damage pickup for Arena mode
	DoubleDamage
};

/**
* Effect actor for animating pickup pads both when idle and when collected.
***********************************************************************************/

UCLASS(Abstract)
class GRIP_API APickupEffect : public AActor
{
	GENERATED_BODY()

public:

	// Construct a pickup effect.
	APickupEffect();

	// The particle system to use for the idle state.
	UPROPERTY(EditAnywhere, Category = Pickup)
		UParticleSystemComponent* IdleEffect = nullptr;

	// The particle system to use for the picked up state.
	UPROPERTY(EditAnywhere, Category = Pickup)
		UParticleSystemComponent* PickedUpEffect = nullptr;

	// Set the location and scale of the pickup effect.
	void SetLocationAndScale(USceneComponent* component, const FVector& location, float scale) const;

	// Handle the visual effects for a pickup collection.
	void OnPickupPadCollected();
};

/**
* Pickup pad actor for vehicles to collect pickups from.
***********************************************************************************/

UCLASS(Abstract, ClassGroup = Pickups)
class GRIP_API APickup : public AActor, public IAttractableInterface, public IPositionableInterface
{
	GENERATED_BODY()

public:

	// Construct a pickup.
	APickup();

	// Position / orient to the nearest surface at run-time.
	UPROPERTY(EditAnywhere, Category = Pickup)
		bool SnapToSurface = true;

	// Should the pickup use the direction of the nearest spline to determine if bots should attempt to pick this up?
	// This should often be switched on for normal racing but off for arena / playground mode.
	UPROPERTY(EditAnywhere, Category = Pickup)
		bool DirectionalCaptureCone = true;

	// Is this just a collectible target rather than a standard pickup.
	UPROPERTY(EditAnywhere, Category = Pickup)
		EPickupClass Class = EPickupClass::Pickup;

	// What kind of pickup to give the player.
	UPROPERTY(EditAnywhere, Category = Pickup)
		EPickupType GivePickup = EPickupType::Random;

	// The offset from the nearest surface in cm.
	UPROPERTY(EditAnywhere, Category = Pickup)
		float SurfaceOffset = 200.0f;

	// The time it takes to spawn a pickup in seconds.
	UPROPERTY(EditAnywhere, Category = Pickup)
		float SpawnTime = 1.0f;

	// The time between collection and respawning in seconds.
	UPROPERTY(EditAnywhere, Category = Pickup)
		float DelayTime = 10.0f;

	// The scale of the particle effects.
	UPROPERTY(EditAnywhere, Category = Pickup)
		float Scale = 1.0f;

	// The distance range, in meters, that will attract AI vehicles towards the pickup.
	UPROPERTY(EditAnywhere, Category = Pickup, meta = (UIMin = "10.0", UIMax = "1000.0", ClampMin = "0.0", ClampMax = "1000.0"))
		float AttractionDistanceRange = 250.0f;

	// The angular range that will attract AI vehicles towards the pickup.
	UPROPERTY(EditAnywhere, Category = Pickup, meta = (UIMin = "10.0", UIMax = "360.0", ClampMin = "0.0", ClampMax = "360.0"))
		float AttractionAngleRange = 30.0f;

	// Only allow AI bots that are on the nearest pursuit spline to this pickup to attempt to pick it up?
	UPROPERTY(EditAnywhere, Category = Pickup)
		bool AttractionPursuitSplineOnly = true;

	// Sound cue for the collected sound.
	UPROPERTY(EditAnywhere, Category = Pickup)
		USoundCue* CollectedSound = nullptr;

	// Sound cue for the collected sound. Non-Player variant.
	UPROPERTY(EditAnywhere, Category = Pickup)
		USoundCue* CollectedSoundNonPlayer = nullptr;

	// Sound cue for the spawned sound.
	UPROPERTY(EditAnywhere, Category = Pickup)
		USoundCue* SpawnedSound = nullptr;

	// Sphere for overlay detection.
	UPROPERTY(EditAnywhere, Category = Pickup)
		USphereComponent* CollisionSphere = nullptr;

	// The mesh to use as a generation pad.
	UPROPERTY(EditAnywhere, Category = Pickup)
		UStaticMeshComponent* PadMesh = nullptr;

	// The effect to use.
	UPROPERTY(EditAnywhere, Category = Pickup)
		TSubclassOf<APickupEffect> Effect;

	// Is the attraction currently active?
	virtual bool IsAttractionActive() const override
	{ return (CurrentState == EState::Uncollected); }

	// Get the attraction location.
	virtual FVector GetAttractionLocation() const override
	{ return AttractionLocation; }

	// Get the attraction direction, or FVector::ZeroVector if no direction.
	virtual FVector GetAttractionDirection() const override
	{ return (DirectionalCaptureCone == true) ? AttractionDirection : FVector::ZeroVector; }

	// Get the attraction distance range from the location.
	virtual float GetAttractionDistanceRange() const override
	{ return AttractionDistanceRangeCms; }

	// Get the attraction minimum distance at which capture can start.
	virtual float GetAttractionMinCaptureDistanceRange() const override
	{ return GetAttractionDistanceRange() * 0.666f; }

	// Get the attraction angle range from the direction.
	virtual float GetAttractionAngleRange() const override
	{ return AttractionAngleRange; }

	enum class EState : uint8
	{
		Uncollected,
		Collected,
		Spawning
	};

private:

	// The current state of the pickup.
	EState CurrentState = EState::Collected;

	// The location of attraction.
	FVector AttractionLocation = FVector::ZeroVector;

	// The direction of attraction, as derived from the nearest pursuit spline.
	FVector AttractionDirection = FVector::ForwardVector;

	// The range of attraction, in centimeters.
	float AttractionDistanceRangeCms = 0.0f;

	// Audio component for the collected sound.
	UPROPERTY(Transient)
		UAudioComponent* CollectedAudio = nullptr;

	// Audio component for the spawned sound.
	UPROPERTY(Transient)
		UAudioComponent* SpawnedAudio = nullptr;

	// The pickup effect.
	UPROPERTY(Transient)
		APickupEffect* PickupEffect = nullptr;
};
