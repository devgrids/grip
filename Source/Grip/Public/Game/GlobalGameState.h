/**
*
* The global state for the entire game. This is persistent state that gets saved
* between levels so the main UI can communicate with a level and vice versa.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Use UGlobalGameState::GetGlobalGameState to access the game state.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "engine/gameinstance.h"
#include "gameframework/playerinput.h"
#include "system/commontypes.h"
#include "kismet/kismetmathlibrary.h"
#include "globalgamestate.generated.h"

/**
* Screen mode.
***********************************************************************************/

UENUM(BlueprintType)
enum class EScreenMode : uint8
{
	FullScreen,
	FullScreenWindow,
	FramedWindow
};

/**
* A screen resolution and associated characteristics.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FScreenResolution
{
	GENERATED_USTRUCT_BODY()

public:

	// The width in pixels.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = ScreenResolution)
		int32 Width = 0;

	// The height in pixels.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = ScreenResolution)
		int32 Height = 0;

	// The refresh rate in Hz.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = ScreenResolution)
		int32 RefreshRate = 0;

	// The display name.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = ScreenResolution)
		FString Description;
};

/**
* Graphics options.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FGraphicsOptions
{
	GENERATED_USTRUCT_BODY()

public:

	// The screen resolution to use.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Graphics)
		FScreenResolution ScreenResolution;

	// The screen mode to use.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		EScreenMode ScreenMode = EScreenMode::FullScreen;

	// The brightness level between 0 and 1.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		float BrightnessLevel = 0.5f;

	// The contrast level between 0 and 1.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		float ContrastLevel = 0.5f;

	// The saturation level between 0 and 1.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		float SaturationLevel = 0.5f;

	// The gamma level between 0 and 1.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		float GammaLevel = 0.5f;

	// The film grain level between 0 and 1.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		float FilmGrain = 0.5f;

	// The anti-aliasing level.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		EAntiAliasingMode AntiAliasing = EAntiAliasingMode::TemporalAA;

	// The anisotropy level.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		EQualityLevel AnisotropyDetail = EQualityLevel::Medium;

	// The motion blur switch.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		EQualityLevel MotionBlur = EQualityLevel::Medium;

	// The speed blur switch.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		EQualityLevel SpeedBlur = EQualityLevel::High;

	// The ambient occlusion switch.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		EOffOnSwitch AmbientOcclusion = EOffOnSwitch::On;

	// The HDR switch.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		EOffOnSwitch HighDynamicRange = EOffOnSwitch::Off;

	// The display nits.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		int32 DisplayNits = 1000;

	// The screen sync switch.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		EOffOnSwitch ScreenSync = EOffOnSwitch::On;

	// The antialiasing detail level.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		EQualityLevel AntiAliasingDetail = EQualityLevel::Epic;

	// The objects detail level.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		EQualityLevel ObjectsDetail = EQualityLevel::Epic;

	// The effects detail level.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		EQualityLevel EffectsDetail = EQualityLevel::Epic;

	// The shadow detail level.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		EQualityLevel ShadowDetail = EQualityLevel::Epic;

	// The texture detail level.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		EQualityLevel TextureDetail = EQualityLevel::Epic;

	// The post processing level.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		EQualityLevel PostProcessingDetail = EQualityLevel::Epic;

	// The resolution scale to apply.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		int32 ResolutionQuality = 100;

	// The limit frame rate switch.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		EOffOnSwitch LimitMaxFPS = EOffOnSwitch::Off;

	// The maximum frames per second, or zero if no maximum.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		int32 MaxFPS = 60;

	// The visibility of the speed streaks.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		float SpeedStreakVisibility = 1.0f;

	// The visibility of the cockpit vehicle.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Graphics)
		float CockpitVehicleVisibility = 1.0f;

	// Get a brightness level to apply directly to the engine graphics settings.
	float GetBrightnessLevel() const
	{ return (0.5f - ((0.5f - BrightnessLevel) * 0.8f)) * 2.0f; }

	// Get a contrast level to apply directly to the engine graphics settings.
	float GetContrastLevel() const
	{ return (0.5f - ((0.5f - ContrastLevel) * 0.8f)) * 2.0f; }

	// Get a gamma level to apply directly to the engine graphics settings.
	float GetGammaLevel() const
	{ return (0.5f - ((0.5f - GammaLevel) * 0.8f)) * 2.0f; }

	// Get a saturation level to apply directly to the engine graphics settings.
	float GetSaturationLevel() const
	{ return SaturationLevel * 2.0f; }
};

/**
* Audio options.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FAudioOptions
{
	GENERATED_USTRUCT_BODY()

public:

	// The effects volume level between 0 and 1.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Audio)
		float EffectsVolumeLevel = 0.7f;

	// The music volume level between 0 and 1.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Audio)
		float MusicVolumeLevel = 0.35f;

	// Mute the sound effects?
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Audio)
		bool MuteEffects = false;

	// Mute the music?
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Audio)
		bool MuteMusic = false;
};

/**
* General options.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FGeneralOptions
{
	GENERATED_USTRUCT_BODY()

public:

	// The level of difficulty, starting from 0 for easiest and up to whatever value we see fit -
	// I would suggest 4 difficulty levels, the highest being extraordinary and the others standard.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = General, meta = (UIMin = "0", UIMax = "3", ClampMin = "0", ClampMax = "3"))
		int32 DifficultyLevel = 0;

	// The speed unit.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = General)
		ESpeedDisplayUnit SpeedUnit = ESpeedDisplayUnit::KPH;

	// Show name tags?
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = General)
		EShowPlayerNameTags ShowPlayerNameTags = EShowPlayerNameTags::All;

	// Use the crash camera?
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = General)
		bool UseCrashCamera = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = General)
		bool InstantaneousLook = false;

	// How much the scale the race camera field-of-view by.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = General, meta = (UIMin = "50", UIMax = "150", ClampMin = "50", ClampMax = "150"))
		float RaceCameraFOVScale = 0.4f;

	// The number of players in a race.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = General, meta = (UIMin = "1", UIMax = "10", ClampMin = "1", ClampMax = "10"))
		int32 NumberOfPlayers = GRIP_MAX_PLAYERS;

	// The number of laps in a race.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = General, meta = (UIMin = "1", UIMax = "20", ClampMin = "1", ClampMax = "20"))
		int32 NumberOfLaps = 4;

	// The maximum game time for arena mode.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = General, meta = (UIMin = "1", UIMax = "20", ClampMin = "1", ClampMax = "20"))
		int32 MaximumGameTime = 5;

	// Are pickups active?
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = General)
		bool PickupsActive = true;

	// Are vehicles destructible?
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = General)
		bool DestructibleVehicles = false;

	// Is the track to be mirrored?
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = General)
		bool MirrorTrack = false;

	// Use catchup assist?
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = General)
		bool CatchupAssist = true;

	// The vehicle engine power level from 0 to 2.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = General, meta = (UIMin = "0", UIMax = "3", ClampMin = "0", ClampMax = "3"))
		int32 EnginePowerLevel = 1;

	// The HUD brightness level between 0 and 1.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = General, meta = (UIMin = "0", UIMax = "1", ClampMin = "0", ClampMax = "1"))
		float HUDBrightnessLevel = 1.0f;

	// The HUD brightness level between 0 and 1.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = General, meta = (UIMin = "0.01", UIMax = "1", ClampMin = "0.01", ClampMax = "1"))
		float HUDSize = 0.5f;

	// Show HUD damage?
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = General)
		EOffOnSwitch HUDDamage = EOffOnSwitch::On;

	// The layout for split-screen.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = General)
		ESplitScreenLayout SplitScreenLayout = ESplitScreenLayout::TwoPlayerVertical;

	// Get the power scale of the engines used by vehicles.
	float GetEnginePowerScale(int32 difficultyLevel, int32 enginePowerLevel = -1) const;

	// Get the drag scale used by vehicles.
	float GetDragScale(int32 difficultyLevel) const;
};

/**
* Input controller options.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FInputControllerOptions
{
	GENERATED_USTRUCT_BODY()

public:

	// The speed of the steering response, between 0 and 1.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Input, meta = (UIMin = "0", UIMax = "1", ClampMin = "0", ClampMax = "1"))
		float AnalogSteeringSpeed = 1.0f;

	// The speed of the steering response, between 0 and 1.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Input, meta = (UIMin = "0", UIMax = "1", ClampMin = "0", ClampMax = "1"))
		float DigitalSteeringSpeed = 0.0f;

	// The sensitivity of the steering response, between 0 and 1.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Input, meta = (UIMin = "0", UIMax = "1", ClampMin = "0", ClampMax = "1"))
		float SteeringSensitivity = 0.0f;

	// The deadzone to apply to analog controllers, between 0 and 1.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Input, meta = (UIMin = "0", UIMax = "1", ClampMin = "0", ClampMax = "1"))
		float AnalogDeadZone = 0.15f;

	// Use force feedback?.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Input)
		bool UseForceFeedback = true;

	// The strength of the force feedback, between 0 and 1.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Input)
		float ForceFeedbackStrength = 1.0f;

	// Ignore right stick look around if a direction key is bound
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = Input)
		bool IgnoreRightStick = false;

	// List of action keys used for the mapping
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Input)
		TArray<FInputActionKeyMapping> ActionMappings;

	// List of axis keys used for the mapping
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Input)
		TArray<FInputAxisKeyMapping> AxisMappings;
};

/**
* The setup for the current game.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FGamePlaySetup
{
	GENERATED_USTRUCT_BODY()

public:

	// Which type of game is being played?
	UPROPERTY(BlueprintReadWrite, Category = GamePlay)
		EGameType GameType = EGameType::SinglePlayerEvent;

	// Which kind of driving should be performed?
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = GamePlay)
		EDrivingMode DrivingMode = EDrivingMode::Race;
};

/**
* Transient game state, never recorded to storage, but persists between levels.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FTransientGameState
{
	GENERATED_USTRUCT_BODY()

public:

	// Was the last game event forcefully quit by the player?
	UPROPERTY(BlueprintReadOnly, Category = Transient)
		bool EventQuit = false;

	// Show the FPS in place of the speedometer?
	UPROPERTY(BlueprintReadWrite, Category = Transient)
		bool ShowFPS = false;

	// The overall brightness of map between 0 and 1.
	UPROPERTY(BlueprintReadOnly, Category = Transient)
		float MapBrightness = 1.0f;

	// The overall surface color of map.
	UPROPERTY(BlueprintReadOnly, Category = Transient)
		FVector MapSurfaceColor = FVector(1.0f, 0.75f, 0.66f);

	// The overall light color of map.
	UPROPERTY(BlueprintReadOnly, Category = Transient)
		FVector MapLightingColor = FVector(1.0f, 1.0f, 1.0f);

	// The race camera positions for players.
	UPROPERTY(BlueprintReadOnly, Category = Transient)
		TArray<int32> RaceCameraPositions;

	// The name of the map resource we're to use for playing the game.
	UPROPERTY(BlueprintReadWrite, Category = Transient)
		FString MapName = FString("None");

	// The name of the navigation layer to use for the current level.
	UPROPERTY(BlueprintReadWrite, Category = Transient)
		FString NavigationLayer;

	// The number of local players in a race.
	UPROPERTY(BlueprintReadWrite, Category = Transient)
		int32 NumberOfLocalPlayers = 1;

	// The distance from the nearest spline at which point a vehicle is considered off-track and auto-teleported.
	UPROPERTY(BlueprintReadWrite, Category = Transient)
		float OffTrackDistance = 0.0f;

	// The distance from the nearest spline at which point a vehicle is considered off-track and auto-teleported.
	UPROPERTY(BlueprintReadWrite, Category = Transient)
		float UnderTrackDistance = 0.0f;
};

/**
* The custom global game state that persists between levels.
***********************************************************************************/

UCLASS()
class GRIP_API UGlobalGameState : public UGameInstance
{
	GENERATED_BODY()

public:

	// Construct a global game state.
	UGlobalGameState();

	// The game play setup.
	UPROPERTY(BlueprintReadWrite, Category = General)
		FGamePlaySetup GamePlaySetup;

	// The general options.
	UPROPERTY(BlueprintReadWrite, Category = General)
		FGeneralOptions GeneralOptions;

	// The audio options.
	UPROPERTY(BlueprintReadWrite, Category = General)
		FAudioOptions AudioOptions;

	// The graphics options.
	UPROPERTY(BlueprintReadWrite, Category = General)
		FGraphicsOptions GraphicsOptions;

	// The input options for each local player, should be GRIP_MAX_LOCAL_PLAYERS in length.
	UPROPERTY(BlueprintReadWrite, Category = General)
		TArray<FInputControllerOptions> InputControllerOptions;

	// Transient game state passed between the UI and the game, not persistent between gaming sessions.
	UPROPERTY(BlueprintReadWrite, Category = General)
		FTransientGameState TransientGameState;

	// Is this game using a split-screen layout?
	UFUNCTION(BlueprintCallable, Category = General)
		bool UsingSplitScreen()
	{ return (TransientGameState.NumberOfLocalPlayers > 1); }

	// Is this game using horizontal split-screen layout?
	UFUNCTION(BlueprintCallable, Category = General)
		bool UsingHorizontalSplitScreen()
	{ return (UsingSplitScreen() == true && GeneralOptions.SplitScreenLayout == ESplitScreenLayout::TwoPlayerHorizontal); }

	// Is this game using vertical split-screen layout?
	UFUNCTION(BlueprintCallable, Category = General)
		bool UsingVerticalSplitScreen()
	{ return (UsingSplitScreen() == true && GeneralOptions.SplitScreenLayout == ESplitScreenLayout::TwoPlayerVertical); }

	// Is the given game mode a racing type?
	UFUNCTION(BlueprintCallable, Category = General)
		static bool IsGivenGameModeRace(EDrivingMode drivingMode)
	{ return (drivingMode == EDrivingMode::Race || drivingMode == EDrivingMode::Elimination); }

	// Is the current game mode a racing type?
	UFUNCTION(BlueprintCallable, Category = General)
		bool IsGameModeRace()
	{ return IsGivenGameModeRace(GamePlaySetup.DrivingMode); }

	// Is the given game mode a lap-based?
	UFUNCTION(BlueprintCallable, Category = General)
		static bool IsGivenGameModeLapBased(EDrivingMode drivingMode)
	{ return (drivingMode == EDrivingMode::Race); }

	// Is the current game mode lap-based?
	UFUNCTION(BlueprintCallable, Category = General)
		bool IsGameModeLapBased()
	{ return IsGivenGameModeLapBased(GamePlaySetup.DrivingMode); }

	// Is the given game mode an arena type?
	UFUNCTION(BlueprintCallable, Category = General)
		static bool IsGivenGameModeArena(EDrivingMode drivingMode)
	{ return false; }

	// Is the current game mode an arena type?
	UFUNCTION(BlueprintCallable, Category = General)
		bool IsGameModeArena()
	{ return IsGivenGameModeArena(GamePlaySetup.DrivingMode); }

	// Is the given game mode a ranked type?
	UFUNCTION(BlueprintCallable, Category = General)
		static bool IsGivenGameModeRanked(EDrivingMode drivingMode)
	{ return (IsGivenGameModeArena(drivingMode) == true); }

	// Is the current game mode ranked rather than based on race position?
	UFUNCTION(BlueprintCallable, Category = General)
		bool IsGameModeRanked()
	{ return IsGivenGameModeRanked(GamePlaySetup.DrivingMode); }

	// Is respawning of vehicles available?
	UFUNCTION(BlueprintCallable, Category = General)
		bool RespawnAvailable()
	{ return (GamePlaySetup.DrivingMode == EDrivingMode::Race); }

	// Are the vehicles destructible in this game?
	UFUNCTION(BlueprintCallable, Category = General)
		bool UsingDestructibleVehicles()
	{ return (GamePlaySetup.DrivingMode == EDrivingMode::Elimination) ? false : GeneralOptions.DestructibleVehicles; }

	// Get the difficulty level of the game between 0 and 3 inclusive.
	UFUNCTION(BlueprintCallable, Category = General)
		int32 GetDifficultyLevel()
	{ return GeneralOptions.DifficultyLevel; }

	// Are we using catchup assist, or rubber banding, in this game?
	UFUNCTION(BlueprintCallable, Category = General)
		bool GetCatchupAssist()
	{ return GeneralOptions.CatchupAssist; }

	// Is this a shipping build of the game?
	UFUNCTION(BlueprintCallable, Category = General)
		static bool IsShippingBuild();

	// Synchronize the input with the editor and the players.
	UFUNCTION(BlueprintCallable, Category = General)
		static void SynchronizeInputMapping(UWorld* world);

	// Set the volume of a particular sound class.
	UFUNCTION(BlueprintCallable, Category = General)
		static void SetSoundClassVolume(USoundClass* soundClass, float volume);

	// Is the track currently mirrored?
	UFUNCTION(BlueprintCallable, Category = General)
		bool IsTrackMirrored() const
	{ return GeneralOptions.MirrorTrack; }

	// Are pickups currently active?
	UFUNCTION(BlueprintCallable, Category = General)
		bool ArePickupsActive()
	{ return (GeneralOptions.PickupsActive == true); }

	// Reset the input for a player.
	UFUNCTION(BlueprintCallable, Category = General)
		bool ResetPlayerInput(int32 playerID, bool fillGaps = false);

	// Get the global game state.
	UFUNCTION(BlueprintCallable, Category = General)
		static UGlobalGameState* GetGlobalGameState(UGameInstance* instance, bool check = true)
	{ ensureMsgf(check == false || instance != nullptr, TEXT("No game instance provided to GetGlobalGameState")); auto result = (Cast<UGlobalGameState>(instance)); return result; }

	// Save the global game state.
	UFUNCTION(BlueprintCallable, Category = General)
		static void SaveGlobalGameState(UGameInstance* instance);

	// Load the global game state.
	UFUNCTION(BlueprintCallable, Category = General)
		static void LoadGlobalGameState(UGameInstance* instance, bool force = false);

	// Return the maximum number of local players.  Will be 2 or 4.
	UFUNCTION(BlueprintCallable, Category = General)
		int32 GetMaxLocalPlayers()
	{ return GRIP_MAX_LOCAL_PLAYERS; }

	// Does a vehicle's antigravity status match the given types?
	UFUNCTION(BlueprintCallable, Category = General)
		static bool VehicleMatchesType(bool antigravity, EVehicleTypes types)
	{
		return (antigravity == false && (types == EVehicleTypes::Both || types == EVehicleTypes::Classic)) ||
			(antigravity == true && (types == EVehicleTypes::Both || types == EVehicleTypes::Antigravity));
	}

	// Has this game state been loaded?
	bool IsLoaded() const
	{ return Loaded; }

	// Sanity-check the input options.
	void VerifyInputOptions(bool fillEmptyMappings);

	// Setup a default resolution derived from cues from the current hardware.
	void SetupDefaultResolution();

	// Get the global game state.
	static UGlobalGameState* GetGlobalGameState(const AActor* actor)
	{ auto result = (Cast<UGlobalGameState>(actor->GetGameInstance())); return result; }

	// Get the global game state.
	static UGlobalGameState* GetGlobalGameState(const UWorld* world)
	{ auto result = (Cast<UGlobalGameState>(world->GetGameInstance())); return result; }

private:

	// Has the global game state been loaded from storage?
	bool Loaded = false;
};

/**
* The saved game state.
***********************************************************************************/

UCLASS()
class GRIP_API USaveGameSetup : public USaveGame
{
	GENERATED_BODY()

public:

	// The name of the player.
	UPROPERTY()
		FString PlayerName;

	// The ID of the player.
	UPROPERTY()
		FString PlayerId;

	// The game play setup.
	UPROPERTY()
		FGamePlaySetup GamePlaySetup;

	// The general options.
	UPROPERTY()
		FGeneralOptions GeneralOptions;

	// The audio options.
	UPROPERTY()
		FAudioOptions AudioOptions;

	// The graphics options.
	UPROPERTY()
		FGraphicsOptions GraphicsOptions;

	// The input options for each local player, should be GRIP_MAX_LOCAL_PLAYERS in length.
	UPROPERTY()
		TArray<FInputControllerOptions> InputControllerOptions;
};
