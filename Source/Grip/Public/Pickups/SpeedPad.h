/**
*
* Speed pad implementation.
*
* Original author: Nicky van de Groep.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Speed pads push the vehicles forwards at higher speed when they traverse over
* them. They inherit from the IAttractableInterface interface and so the AI bot
* code will automatically detect them and head towards them where appropriate.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "gameframework/actor.h"
#include "components/staticmeshcomponent.h"
#include "system/attractable.h"
#include "speedpad.generated.h"

/**
* Actor class for a speed pad to give speed boosts to vehicles.
***********************************************************************************/

UCLASS(Abstract)
class GRIP_API ASpeedPad : public AActor, public IAttractableInterface
{
	GENERATED_BODY()

public:

	// Sets default values for this actor's properties.
	ASpeedPad();

	// The amount of additional power the vehicle gets when getting over a speed pad, 1 for 100% extra jet power.
	UPROPERTY(EditAnywhere, Category = SpeedPad, meta = (UIMin = "0.1", UIMax = "2", ClampMin = "0.1", ClampMax = "10"))
		float Power = 1.0f;

	// The duration the vehicle gets a speedup for, in seconds.
	UPROPERTY(EditAnywhere, Category = SpeedPad, meta = (UIMin = "0.1", UIMax = "10", ClampMin = "0.1", ClampMax = "10"))
		float Duration = 1.0f;

	// Sound cue for the collected sound.
	UPROPERTY(EditAnywhere, Category = SpeedPad)
		USoundCue* CollectedSoundPlayer = nullptr;

	// Sound cue for the collected sound of a non player.
	UPROPERTY(EditAnywhere, Category = SpeedPad)
		USoundCue* CollectedSoundNonPlayer = nullptr;

	// The effect to play when the pad is collected by a vehicle.
	UPROPERTY(EditAnywhere, Category = SpeedPad)
		UParticleSystem* CollectedVisual = nullptr;

	// Sphere for detecting when to activate the boost.
	UPROPERTY(EditAnywhere, Category = SpeedPad)
		UBoxComponent* CollisionBox = nullptr;

	// The mesh to use as a generation pad.
	UPROPERTY(EditAnywhere, Category = SpeedPad)
		UStaticMeshComponent* PadMesh = nullptr;

	// The distance range, in meters, that will attract AI vehicles towards the pickup.
	UPROPERTY(EditAnywhere, Category = SpeedPad, meta = (UIMin = "10.0", UIMax = "1000.0", ClampMin = "0.0", ClampMax = "1000.0"))
		float AttractionDistanceRange = 150.0f;

	// The angular range that will attract AI vehicles towards the pickup.
	UPROPERTY(EditAnywhere, Category = SpeedPad, meta = (UIMin = "10.0", UIMax = "360.0", ClampMin = "0.0", ClampMax = "360.0"))
		float AttractionAngleRange = 10.0f;

	// Is the attraction currently active?
	virtual bool IsAttractionActive() const override
	{ return true; }

	// Get the attraction location.
	virtual FVector GetAttractionLocation() const override
	{ return AttractionLocation; }

	// Get the attraction direction, or FVector::ZeroVector if no direction.
	virtual FVector GetAttractionDirection() const override
	{ return AttractionDirection; }

	// Get the attraction distance range from the location.
	virtual float GetAttractionDistanceRange() const override
	{ return AttractionDistanceRangeCms; }

	// Get the attraction minimum distance at which capture can start.
	virtual float GetAttractionMinCaptureDistanceRange() const override
	{ return GetAttractionDistanceRange() * 0.666f; }

	// Get the attraction angle range from the direction.
	virtual float GetAttractionAngleRange() const override
	{ return AttractionAngleRange; }

private:

	// The location of attraction, as derived from the nearest pursuit spline.
	FVector AttractionLocation = FVector::ZeroVector;

	// The direction of attraction, as derived from the nearest pursuit spline.
	FVector AttractionDirection = FVector::ZeroVector;

	// The range of attraction, in centimeters.
	float AttractionDistanceRangeCms = 0.0f;

	// Audio component for the collected sound
	UPROPERTY(Transient)
		UAudioComponent* CollectedAudio = nullptr;

	// The effect to play when the pad is collected by a vehicle.
	UPROPERTY(Transient)
		UParticleSystemComponent* CollectedEffect = nullptr;
};
