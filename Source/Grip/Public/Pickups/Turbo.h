/**
*
* Firestorm turbo implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Turbo pickup type, one of the pickups used by vehicles in the game.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "pickupbase.h"
#include "turbo.generated.h"

struct FPlayerPickupSlot;

/**
* The turbo pickup actor.
***********************************************************************************/

UCLASS(Abstract, ClassGroup = Pickups)
class GRIP_API ATurbo : public APickupBase
{
	GENERATED_BODY()

public:

	// Construct a turbo.
	ATurbo();

	// The strength of the turbo.
	UPROPERTY(EditAnywhere, Category = Turbo)
		FRuntimeFloatCurve BoostVsTime;

	// How much to scale grip by generally to add fear factor.
	UPROPERTY(EditAnywhere, Category = Turbo, meta = (UIMin = "0.1", UIMax = "2.0", ClampMin = "0.1", ClampMax = "2.0"))
		float GripScale = 1.0f;

	// How much to raise the front of the vehicle by to add fear factor.
	UPROPERTY(EditAnywhere, Category = Turbo, meta = (UIMin = "0.0", UIMax = "2.0", ClampMin = "0.0", ClampMax = "2.0"))
		float RaiseFrontScale = 0.0f;

	// Initial delay time for the active sound, in seconds.
	UPROPERTY(EditAnywhere, Category = Turbo, meta = (UIMin = "0.0", UIMax = "5.0", ClampMin = "0.0", ClampMax = "5.0"))
		float ActiveSoundDelayTime = 0.5f;

	// Fade in time for the active sound, in seconds.
	UPROPERTY(EditAnywhere, Category = Turbo, meta = (UIMin = "0.0", UIMax = "5.0", ClampMin = "0.0", ClampMax = "5.0"))
		float ActiveSoundFadeInTime = 0.333f;

	// Fade out time for the active sound, in seconds.
	UPROPERTY(EditAnywhere, Category = Turbo, meta = (UIMin = "0.0", UIMax = "5.0", ClampMin = "0.0", ClampMax = "5.0"))
		float ActiveSoundFadeOutTime = 1.0f;

	// Sound cue for the activate sound.
	UPROPERTY(EditAnywhere, Category = Turbo)
		USoundCue* ActivateSound = nullptr;

	// Sound cue for the activate sound. Non-Player variant.
	UPROPERTY(EditAnywhere, Category = Turbo)
		USoundCue* ActivateSoundNonPlayer = nullptr;

	// Sound cue for the active sound.
	UPROPERTY(EditAnywhere, Category = Turbo)
		USoundCue* ActiveSound = nullptr;

	// Sound cue for the active sound. Non-Player variant.
	UPROPERTY(EditAnywhere, Category = Turbo)
		USoundCue* ActiveSoundNonPlayer = nullptr;

	// Looped audio for then the turbo is active.
	UPROPERTY(Transient)
		UAudioComponent* ActiveAudio = nullptr;
};
