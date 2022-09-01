/**
*
* Vehicle audio implementation, use to control main audio on a vehicle.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* All of the audio plumbing required to have your vehicles making noise.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "sound/soundcue.h"
#include "vehicleaudio.generated.h"

#pragma region MinimalVehicle

/**
* Some macros to help access audio components on a vehicle.
***********************************************************************************/

#define GRIP_VEHICLE_AUDIO_JE_IDLE 0
#define GRIP_VEHICLE_AUDIO_JE_THRUST 1
#define GRIP_VEHICLE_AUDIO_PE_IDLE 0
#define GRIP_VEHICLE_AUDIO_PE_GEAR1 1
#define GRIP_VEHICLE_AUDIO_PE_GEAR2 2

#define GRIP_VEHICLE_AUDIO_GEAR_C(x) (((x) == 0) ? GRIP_VEHICLE_AUDIO_PE_GEAR1 : GRIP_VEHICLE_AUDIO_PE_GEAR2)

#define GRIP_STOP_IF_PLAYING(audio) if (audio->IsPlaying() == true) { audio->Stop(); }
#define GRIP_PLAY_IF_NOT_PLAYING(audio) if (audio->IsPlaying() == false) { audio->Play(); }

#define SET_VEHICLE_SOUND_NON_SPATIALIZED(sound) if (sound != nullptr && sound->AttenuationSettings != nullptr) { sound->AttenuationSettings->Attenuation.OmniRadius = FMath::Max(sound->AttenuationSettings->Attenuation.OmniRadius, 75.0f); }

/**
* Audio data for a particular gear in a vehicle's engine.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FVehicleAudioGear
{
	GENERATED_USTRUCT_BODY()

public:

	// Sound cue for the gear up change.
	UPROPERTY(EditAnywhere, Category = Gear)
		USoundCue* ChangeUpSound;

	// Sound cue for the gear down change.
	UPROPERTY(EditAnywhere, Category = Gear)
		USoundCue* ChangeDownSound;

	// Sound cue for the engine in this gear.
	UPROPERTY(EditAnywhere, Category = Gear)
		USoundCue* EngineSound;

	// Minimum pitch for the engine in this gear.
	UPROPERTY(EditAnywhere, Category = Gear)
		float MinEnginePitch;

	// Maximum pitch for the engine in this gear.
	UPROPERTY(EditAnywhere, Category = Gear)
		float MaxEnginePitch;
};

/**
* Audio data for a particular vehicle.
***********************************************************************************/

UCLASS(ClassGroup = Audio)
class UVehicleAudio : public UDataAsset
{
	GENERATED_BODY()

public:

	// Sound cue for a hard landing on a wheel's suspension.
	UPROPERTY(EditAnywhere, Category = Impact)
		USoundCue* HardLandingSound = nullptr;

	// Sound cue for the jet engine idle sound. Non-player variant.
	UPROPERTY(EditAnywhere, Category = JetEngine)
		USoundCue* JetEngineIdleSound = nullptr;

	// Sound cue for the jet engine sound.
	UPROPERTY(EditAnywhere, Category = JetEngine)
		USoundCue* JetEngineSound = nullptr;

	// Minimum pitch for the jet engine.
	UPROPERTY(EditAnywhere, Category = JetEngine)
		float MinJetEnginePitch = 0.5f;

	// Maximum pitch for the jet engine.
	UPROPERTY(EditAnywhere, Category = JetEngine)
		float MaxJetEnginePitch = 2.0f;

	// Maximum speed for the jet engine, at which is should achieve maximum pitch, in KPH.
	UPROPERTY(EditAnywhere, Category = JetEngine)
		float MaxJetEngineSpeed = 500.0f;

	// Sound cue for the engine idle sound.
	UPROPERTY(EditAnywhere, Category = PistonEngine)
		USoundCue* EngineIdleSound = nullptr;

	// Initial delay time for a new gear in the piston engine, in seconds.
	UPROPERTY(EditAnywhere, Category = PistonEngine)
		float EngineSoundDelayTime = 0.5f;

	// Fade in time for a new gear in the piston engine, in seconds.
	UPROPERTY(EditAnywhere, Category = PistonEngine)
		float EngineSoundFadeInTime = 0.333f;

	// Fade out time for an old gear in the piston engine, in seconds.
	UPROPERTY(EditAnywhere, Category = PistonEngine)
		float EngineSoundFadeOutTime = 0.83f;

	// Sound cue for the engine boost sound.
	UPROPERTY(EditAnywhere, Category = PistonEngine)
		USoundCue* EngineBoostSound = nullptr;

	// The gears for the piston engine.
	UPROPERTY(EditAnywhere, Category = PistonEngine)
		TArray<FVehicleAudioGear> Gears;
};

#pragma endregion MinimalVehicle
