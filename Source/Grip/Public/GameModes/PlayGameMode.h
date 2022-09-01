/**
*
* Play game mode implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* The play game mode to use for the game, specifically for playing a level and
* is the C++ game mode used in GRIP, with a blueprint wrapping it for actual use.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "ai/trackcheckpoint.h"
#include "system/timesmoothing.h"
#include "system/mathhelpers.h"
#include "system/timeshareclock.h"
#include "system/avoidable.h"
#include "gamemodes/basegamemode.h"
#include "effects/drivingsurfacecharacteristics.h"
#include "pickups/pickup.h"
#include "playgamemode.generated.h"

struct FPlayerPickupSlot;

class UWidget;
class UPursuitSplineComponent;
class ABaseVehicle;
class ASpeedPad;
class AHomingMissile;
class AStaticTrackCamera;
class APursuitSplineActor;
class USingleHUDWidget;

/**
* Which part of the game sequence is the current game in?
***********************************************************************************/

UENUM(BlueprintType)
enum class EGameSequence : uint8
{
	None,
	Initialise,
	Start,
	Play,
	End
};

/**
* Types for game events, events being actions happening within a game.
***********************************************************************************/

UENUM(BlueprintType)
enum class EGameEventType : uint8
{
	Unknown,
	Preparing,
	Used,
	Blocked,
	Incoming,
	Impacted,
	Destroyed,
	ChatMessage
};

/**
* Game events, events being actions happening within a game.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FGameEvent
{
	GENERATED_USTRUCT_BODY()

public:

	// The time the event happened.
	UPROPERTY(Transient, BlueprintReadOnly, Category = Event)
		float Time = 0.0f;

	// The index number of the actioning vehicle.
	UPROPERTY(Transient, BlueprintReadOnly, Category = Event)
		int32 LaunchVehicleIndex = -1;

	// The index number of the actioned to vehicle.
	UPROPERTY(Transient, BlueprintReadOnly, Category = Event)
		int32 TargetVehicleIndex = -1;

	// The pickup used in this event, if any.
	UPROPERTY(Transient, BlueprintReadOnly, Category = Event)
		EPickupType PickupUsed = EPickupType::None;

	// Was the pickup used in this event charged?
	UPROPERTY(Transient, BlueprintReadOnly, Category = Event)
		bool PickupUsedWasCharged = false;

	// The type of the event.
	UPROPERTY(Transient, BlueprintReadOnly, Category = Event)
		EGameEventType EventType = EGameEventType::Unknown;

	// Additional textual information for this event.
	UPROPERTY(Transient, BlueprintReadOnly, Category = Event)
		FString ExtraInformation;
};

/**
* Characteristics used to describe vehicle catchup, or rubber banding.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FVehicleCatchupCharacteristics
{
	GENERATED_USTRUCT_BODY()

public:

	// Over what area, in meters, should be we trying to keep the pack within?
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "1.0", UIMax = "2500.0", ClampMin = "1.0", ClampMax = "2500.0"))
		float DistanceSpread = 500.0f;

	// What distance, in meters, should be we aim to congregate AI vehicles away from the center distance of the all the human players?
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "-2500.0", UIMax = "2500.0", ClampMin = "-2500.0", ClampMax = "2500.0"))
		float CentreOffset = 0.0f;

	// The delay factor for effecting speed change.
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "10.0", ClampMin = "0.0", ClampMax = "10.0"))
		float SpeedChangeDelay = 3.0f;

	// How much to scale drag by when leading the pack, larger numbers mean MORE drag.
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "1.0", ClampMin = "0.0", ClampMax = "1.0"))
		float DragScaleAtFrontHumans = 0.5f;

	// How much to scale drag by when leading the pack, larger numbers mean MORE drag.
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "1.0", ClampMin = "0.0", ClampMax = "1.0"))
		float DragScaleAtFrontNonHumans = 0.5f;

	// How much to scale drag by when trailing the pack, larger numbers mean LESS drag.
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "1.0", ClampMin = "0.0", ClampMax = "1.0"))
		float DragScaleAtRearHumans = 0.25f;

	// How much to scale drag by when trailing the pack, larger numbers mean LESS drag.
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "1.0", ClampMin = "0.0", ClampMax = "1.0"))
		float DragScaleAtRearNonHumans = 0.25f;

	// How much to scale grip by when leading the pack, larger numbers mean LESS grip.
	// We never apply this to humans as it's too unfair, but a useful device to get AI cars to screw up when leading.
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "1.0", ClampMin = "0.0", ClampMax = "1.0"))
		float GripScaleAtFrontNonHumans = 0.0f;

	// How much to scale grip by when trailing the pack, larger numbers mean MORE grip.
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "1.0", ClampMin = "0.0", ClampMax = "1.0"))
		float GripScaleAtRearHumans = 0.25f;

	// How much to scale grip by when trailing the pack, larger numbers mean MORE grip.
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "1.0", ClampMin = "0.0", ClampMax = "1.0"))
		float GripScaleAtRearNonHumans = 0.25f;

	// How much to improve low-speed acceleration by when trailing the pack, larger numbers mean MORE acceleration.
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "1.0", ClampMin = "0.0", ClampMax = "1.0"))
		float LowSpeedAccelerationScaleAtRear = 0.25f;
};

/**
* Characteristics used to describe weapon catchup, or how bots use weapons based
* on their position in the pack.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FWeaponCatchupCharacteristics
{
	GENERATED_USTRUCT_BODY()

public:

	// The trailing distance to timidly use offensive weapons against unknown opponents.
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "2500.0", ClampMin = "0.0", ClampMax = "2500.0"))
		float TrailingDistance = 50.0f;

	// The leading distance to aggressively use offensive weapons against unknown opponents.
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "2500.0", ClampMin = "0.0", ClampMax = "2500.0"))
		float LeadingDistance = 250.0f;

	// The trailing distance a human player is at to timidly use offensive weapons them.
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "2500.0", ClampMin = "0.0", ClampMax = "2500.0"))
		float TrailingDistanceHumans = 100.0f;

	// The leading distance a human player is at to aggressively use offensive weapons them.
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "2500.0", ClampMin = "0.0", ClampMax = "2500.0"))
		float LeadingDistanceHumans = 250.0f;

	// The trailing distance a non-human player is at to timidly use offensive weapons them.
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "2500.0", ClampMin = "0.0", ClampMax = "2500.0"))
		float TrailingDistanceNonHumans = 100.0f;

	// The leading distance a non-human player is at to aggressively use offensive weapons them.
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "2500.0", ClampMin = "0.0", ClampMax = "2500.0"))
		float LeadingDistanceNonHumans = 250.0f;
};

/**
* Characteristics used to describe general pickup use.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FPickupUseCharacteristics
{
	GENERATED_USTRUCT_BODY()

public:

	// The minimum delay before a pickup can be used after being picked-up in seconds. (used)
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "1000.0", ClampMin = "0.0", ClampMax = "1000.0"))
		float PickupUseAfter = 2.0f;

	// The ideal time before which a pickup should be used after being picked-up in seconds. (used)
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "1000.0", ClampMin = "0.0", ClampMax = "1000.0"))
		float PickupUseBefore = 30.0f;

	// The time after which a pickup should be dumped if it can't be used effectively after being picked-up in seconds. (used)
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "1000.0", ClampMin = "0.0", ClampMax = "1000.0"))
		float PickupDumpAfter = 0.0f;

	// The responsive to being attacked should we be, with 1 being perfectly and 0 being poorly. (used)
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "1.0", ClampMin = "0.0", ClampMax = "1.0"))
		float DefenseResponsiveness = 1.0f;

	// How often a human should be attacked in seconds. (used)
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "1000.0", ClampMin = "0.0", ClampMax = "1000.0"))
		float MaxHumanAttackFrequency = 20.0f;

	// How often a human should be attacked compared to bots. (used)
	UPROPERTY(EditAnywhere, Category = "System", meta = (UIMin = "0.0", UIMax = "1.0", ClampMin = "0.0", ClampMax = "1.0"))
		float HumanAttackBias = 0.0f;
};

/**
* Characteristics used to describe pickup use separately for race modes.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FGameModesPickupUseCharacteristics
{
	GENERATED_USTRUCT_BODY()

public:

	// The pickup distribution ratios for races.
	UPROPERTY(EditAnywhere, Category = "System")
		FPickupUseCharacteristics Race;
};

/**
* Characteristics used to describe pickup assignment probabilities, with higher
* numbers making pickups more likely to be assigned.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FPickupAssignmentRatios
{
	GENERATED_USTRUCT_BODY()

public:

	// Ratio of shields.
	UPROPERTY(EditAnywhere, Category = "System")
		float Shield = 1.0f;

	// Ratio of turbo boosts.
	UPROPERTY(EditAnywhere, Category = "System")
		float TurboBoost = 1.0f;

	// Ratio of homing missiles.
	UPROPERTY(EditAnywhere, Category = "System")
		float HomingMissile = 1.0f;

	// Ratio of Gatling guns.
	UPROPERTY(EditAnywhere, Category = "System")
		float GatlingGun = 1.0f;
};

/**
* Characteristics used to describe limits on pickup assignment.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FPickupAssignmentMaximums
{
	GENERATED_USTRUCT_BODY()

public:

	// Maximum shields present in the world, either in use or held in pickup slots.
	UPROPERTY(EditAnywhere, Category = "System")
		int32 ShieldMaxPresent = 0;

	// Maximum number of times consecutive assignment of a shield can repeat.
	UPROPERTY(EditAnywhere, Category = "System")
		int32 ShieldMaxRepeat = 2;

	// The minimum number of seconds that should pass by between assigning shields.
	UPROPERTY(EditAnywhere, Category = "System")
		int32 ShieldMinSeconds = 0;

	// Maximum turbos present in the world, either in use or held in pickup slots.
	UPROPERTY(EditAnywhere, Category = "System")
		int32 TurboBoostMaxPresent = 0;

	// Maximum number of times consecutive assignment of a turbo can repeat.
	UPROPERTY(EditAnywhere, Category = "System")
		int32 TurboBoostMaxRepeat = 2;

	// The minimum number of seconds that should pass by between assigning turbos.
	UPROPERTY(EditAnywhere, Category = "System")
		int32 TurboBoostMinSeconds = 0;

	// Maximum homing missiles present in the world, either in use or held in pickup slots.
	UPROPERTY(EditAnywhere, Category = "System")
		int32 HomingMissileMaxPresent = 0;

	// Maximum number of times consecutive assignment of a homing missile can repeat.
	UPROPERTY(EditAnywhere, Category = "System")
		int32 HomingMissileMaxRepeat = 2;

	// The minimum number of seconds that should pass by between assigning homing missiles.
	UPROPERTY(EditAnywhere, Category = "System")
		int32 HomingMissileMinSeconds = 0;

	// Maximum Gatling guns present in the world, either in use or held in pickup slots.
	UPROPERTY(EditAnywhere, Category = "System")
		int32 GatlingGunMaxPresent = 0;

	// Maximum number of times consecutive assignment of a Gatling gun can repeat.
	UPROPERTY(EditAnywhere, Category = "System")
		int32 GatlingGunMaxRepeat = 2;

	// The minimum number of seconds that should pass by between assigning Gatling guns.
	UPROPERTY(EditAnywhere, Category = "System")
		int32 GatlingGunMinSeconds = 0;
};

/**
* Characteristics used to describe pickup assignment for different the sections of
* the vehicle pack.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FSectionPickupAssignmentRatios
{
	GENERATED_USTRUCT_BODY()

public:

	// Ratios for leading vehicles.
	UPROPERTY(EditAnywhere, Category = "System")
		FPickupAssignmentRatios Leading;

	// Ratios for central vehicles.
	UPROPERTY(EditAnywhere, Category = "System")
		FPickupAssignmentRatios Central;

	// Ratios for trailing vehicles.
	UPROPERTY(EditAnywhere, Category = "System")
		FPickupAssignmentRatios Trailing;

	UPROPERTY(EditAnywhere, Category = "System")
		FPickupAssignmentMaximums PickupMaximums;
};

/**
* Characteristics used to describe pickup assignment separate for race modes.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FGameModesPickupAssignmentRatios
{
	GENERATED_USTRUCT_BODY()

public:

	// The pickup distribution ratios for races.
	UPROPERTY(EditAnywhere, Category = "System")
		FSectionPickupAssignmentRatios Race;
};

/**
* All of the combined characteristics for vehicle catchup, weapon catchup,
* pickup use and assignment, grouped together for a difficulty level. All of these
* characteristics can then be tuned for each, individual difficulty level.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FDifficultyCharacteristics
{
	GENERATED_USTRUCT_BODY()

public:

	// The vehicle drag catchup characteristics.
	UPROPERTY(EditAnywhere, Category = "System")
		FVehicleCatchupCharacteristics VehicleCatchupCharacteristics;

	// The non-human weapon-use catchup characteristics.
	UPROPERTY(EditAnywhere, Category = "System")
		FWeaponCatchupCharacteristics WeaponCatchupCharacteristics;

	// The non-human general pickup-use characteristics.
	UPROPERTY(EditAnywhere, Category = "System")
		FGameModesPickupUseCharacteristics PickupUseCharacteristics;

	// The pickup distribution ratios for each game mode.
	UPROPERTY(EditAnywhere, Category = "System")
		FGameModesPickupAssignmentRatios PickupAssignmentRatios;
};

/**
* Overrides, just used for easy testing, not present in shipping builds.
***********************************************************************************/

UCLASS(ClassGroup = GameMode)
class UGameStateOverrides : public UDataAsset
{
	GENERATED_BODY()

public:

	UPROPERTY(EditAnywhere, Category = "Game State Overrides")
		bool OverrideGamePlaySetup = false;

	UPROPERTY(EditAnywhere, Category = "Game State Overrides", meta = (EditCondition = "OverrideGamePlaySetup"))
		FGamePlaySetup GamePlaySetup;

	UPROPERTY(EditAnywhere, Category = "Game State Overrides")
		bool OverrideGeneralOptions = false;

	UPROPERTY(EditAnywhere, Category = "Game State Overrides", meta = (EditCondition = "OverrideGeneralOptions"))
		FGeneralOptions GeneralOptions;

	UPROPERTY(EditAnywhere, Category = "Game State Overrides")
		bool OverrideGraphicsOptions = false;

	UPROPERTY(EditAnywhere, Category = "Game State Overrides", meta = (EditCondition = "OverrideGraphicsOptions"))
		FGraphicsOptions GraphicsOptions;

	UPROPERTY(EditAnywhere, Category = "Game State Overrides")
		bool OverrideAudioOptions = false;

	UPROPERTY(EditAnywhere, Category = "Game State Overrides", meta = (EditCondition = "OverrideAudioOptions"))
		FAudioOptions AudioOptions;

	UPROPERTY(EditAnywhere, Category = "Game State Overrides")
		bool OverrideNavigationLayer = false;

	UPROPERTY(EditAnywhere, Category = "Game State Overrides", meta = (EditCondition = "OverrideNavigationLayer"))
		FString NavigationLayer;

	UPROPERTY(EditAnywhere, Category = "Game State Overrides")
		bool OverrideGrid = false;

	UPROPERTY(EditAnywhere, Category = "Game State Overrides", meta = (EditCondition = "OverrideGrid"))
		TArray<TSubclassOf<ABaseVehicle>> Grid;

	UPROPERTY(EditAnywhere, Category = "Game State Overrides")
		bool SeriousBotBehaviour = false;
};

/**
* The play game mode to use for the game, specifically for playing a level and
* is the C++ game mode used in GRIP, with a blueprint wrapping it for actual use.
***********************************************************************************/

UCLASS(Abstract, Blueprintable)
class GRIP_API APlayGameMode : public ABaseGameMode
{
	GENERATED_BODY()

public:

	// Construct a play game mode.
	APlayGameMode();

	// All of the global game state overrides used for testing.
	UPROPERTY(EditAnywhere, Category = "System")
		UGameStateOverrides* GameStateOverrides = nullptr;

	// The difficulty characteristics for easy mode.
	UPROPERTY(EditAnywhere, Category = "Difficulty")
		FDifficultyCharacteristics DifficultyEasy;

	// The difficulty characteristics for medium mode.
	UPROPERTY(EditAnywhere, Category = "Difficulty")
		FDifficultyCharacteristics DifficultyMed;

	// The difficulty characteristics for hard mode.
	UPROPERTY(EditAnywhere, Category = "Difficulty")
		FDifficultyCharacteristics DifficultyHard;

	// The difficulty characteristics for Neo mode.
	UPROPERTY(EditAnywhere, Category = "Difficulty")
		FDifficultyCharacteristics DifficultyNeo;

	// The widget to use for the single screen UI.
	UPROPERTY(Transient)
		USingleHUDWidget* SingleScreenWidget;

	// Is the game currently paused?
	UPROPERTY(Transient, BlueprintReadOnly, Category = "System")
		bool GamePaused = false;

	// Should the AI bots slam on the brakes and stop what they're doing?
	UPROPERTY(Transient, BlueprintReadWrite, Category = "System")
		bool StopWhatYouDoing = false;

	// The events that have occurred during this particular game event.
	UPROPERTY(Transient, BlueprintReadOnly, Category = "System")
		TArray<FGameEvent> GameEvents;

	// The vehicles currently present in the game.
	UPROPERTY(Transient)
		TArray<ABaseVehicle*> Vehicles;

	// The homing missiles currently present in the game.
	UPROPERTY(Transient)
		TArray<AHomingMissile*> Missiles;

	// The track cameras contained in the current level.
	UPROPERTY(Transient)
		TArray<AStaticTrackCamera*> TrackCameras;

	// The speed pads contained in the current level. Currently unused, but left in any way.
	UPROPERTY(Transient)
		TArray<ASpeedPad*> SpeedPads;

	// The pickup pads contained in the current level. Currently unused, but left in any way.
	UPROPERTY(Transient)
		TArray<APickup*> PickupPads;

	// The pursuit splines contained in the current level.
	UPROPERTY(Transient)
		TArray<APursuitSplineActor*> PursuitSplines;

	// The checkpoints contained in the current level.
	UPROPERTY(Transient)
		TArray<ATrackCheckpoint*> Checkpoints;

	// Get the text that describes the XP associated with the use of a pickup.
	UFUNCTION(BlueprintImplementableEvent, Category = UI)
		FText GetXPMessage(EPickupType pickupType, int32 numPoints);

	// Update the player tags on the HUD.
	UFUNCTION(BlueprintCallable, Category = Default)
		void UpdatePlayerTags(APawn* owningPawn, UPanelWidget* parent);

	// Are we past the game sequence start?
	UFUNCTION(BlueprintCallable, Category = "System")
		bool PastGameSequenceStart() const
	{ return (GameSequence > EGameSequence::Start); }

	// The default ChoosePlayerStart is broken in the engine, so we override it here to allocate player starts serially to vehicles.
	UFUNCTION(BlueprintCallable, Category = General)
		AActor* ChoosePlayerStartProperly(AController* player, int32 maxPlayers);

	// Get the current opacity for a flashing state, used to flash element on / off on the HUD.
	UFUNCTION(BlueprintCallable, Category = UI)
		float GetFlashingOpacity(bool rapid = false) const
	{ float scale = (rapid == true) ? 2.0f : 1.0f;  return ((FMath::Fmod(GetRealTimeClock() * scale, 1.0f) < 0.5f) ? 1.0f : 0.0f); }

	// Get the difficulty characteristics for a given level, or the current level if -1 is passed.
	UFUNCTION(BlueprintCallable, Category = UI)
		FDifficultyCharacteristics& GetDifficultyCharacteristics(int32 level = -1);

	// Get the vehicle that is the current camera target.
	ABaseVehicle* CameraTarget(int32 localPlayerIndex);

	// Get the countdown time for the race.
	FText GetCountDownTime() const;

	// Get the number of players dead or alive in the game.
	int32 GetNumOpponents(bool humansOnly = false);

	// Get the number of players left in the game.
	int32 GetNumOpponentsLeft() const
	{ return Vehicles.Num() - NumPlayersDestroyed; }

	// Get the scale of the HUD.
	float GetHUDScale() const;

	// Has the game come to an end in terms of its sequencing?
	bool GameHasEnded() const
	{ return (GameSequence >= EGameSequence::End); }

	// Get the countdown opacity for the text at the start of a race.
	float GetCountdownOpacity() const;

	// Get the vehicle for a vehicle index.
	ABaseVehicle* GetVehicleForVehicleIndex(int32 vehicleIndex) const;

	// Get the vehicles currently present in the game.
	TArray<ABaseVehicle*>& GetVehicles()
	{ if (Vehicles.Num() == 0) DetermineVehicles(); return Vehicles; }

	// Get the pursuit splines currently present in the game.
	TArray<APursuitSplineActor*>& GetPursuitSplines()
	{ if (PursuitSplines.Num() == 0) DeterminePursuitSplines(); return PursuitSplines; }

	// Get the amount of time since the game ended.
	float GetGameEndedClock() const
	{ return (GameSequence == EGameSequence::End) ? GetRealTimeClock() - GameFinishedAt : 0.0f; }

	// Have all the players finished the event.
	bool HaveAllPlayersFinished() const;

	// Quit the game.
	void QuitGame(bool force = false);

	// Get the ratio of completion for the current event, 1 being fully complete.
	float GetEventProgress();

	// Restart the game.
	virtual void RestartGame() override;

	// Project a point in world space for use on the HUD.
	bool ProjectWorldLocationToWidgetPosition(APawn* pawn, FVector worldLocation, FVector2D& screenPosition, FMinimalViewInfo* cachedView = nullptr);

	// Are there no opponents left in this game?
	bool NoOpponentsLeft() const
	{ return ((Vehicles.Num() - NumPlayersDestroyed) < 2); }

	// Record the destruction of a player for this game.
	void DestroyPlayer(ABaseVehicle* vehicle)
	{ NumPlayersDestroyed++; }

	// Get the time left before the game starts.
	float GetPreStartTime() const;

	// Get the vehicle at a given index into the vehicle list.
	ABaseVehicle* GetVehicle(int32 index) const
	{ return (Vehicles.IsValidIndex(index) == true) ? Vehicles[index] : nullptr; }

	// Get the ratio between 0 and 1 until the next elimination occurs in an Elimination game.
	// 0 meaning the point of elimination.
	float GetEliminationRatio() const
	{ float difference = GRIP_ELIMINATION_SECONDS - EliminationTimer; return (difference < GRIP_ELIMINATION_WARNING_SECONDS) ? 1.0f - (difference / GRIP_ELIMINATION_WARNING_SECONDS) : 0.0f; }

	// Get the timer used for elimination, always incrementing upwards.
	float GetEliminationTimer() const
	{ return EliminationTimer; }

	// Attractables is a map because looking up an interface at run-time is expensive.
	TMap<AActor*, IAttractableInterface*> Attractables;

	// Avoidables is a map because looking up an interface at run-time is expensive.
	TMap<AActor*, IAvoidableInterface*> Avoidables;

	// Collect a race finishing position when a player crosses the line.
	int32 CollectFinishingRacePosition()
	{ return FMath::Min(NextFinishingRacePosition++, GRIP_MAX_PLAYERS - 1); }

	// Collect a death position when a player is eliminated from a game.
	int32 CollectDeathPosition()
	{ return (Vehicles.Num() - 1) - NextDeathPosition++; }

	// Get a random player start point.
	APlayerStart* GetRandomPlayerStart() const;

	// Add an avoidable to the list of avoidables present in the current level.
	void AddAvoidable(AActor* actor)
	{ if (Avoidables.Contains(actor) == false) Avoidables.Emplace(actor, Cast<IAvoidableInterface>(actor)); }

	// Remove an avoidable from the list of avoidables present in the current level.
	void RemoveAvoidable(AActor* actor)
	{ if (Avoidables.Contains(actor) == true) { Avoidables.Remove(actor); Avoidables.Compact(); } }

	// Add an attractable to the list of attractables present in the current level.
	void AddAttractable(AActor* actor)
	{ if (Attractables.Contains(actor) == false) Attractables.Emplace(actor, Cast<IAttractableInterface>(actor)); }

	// Remove an attractable from the list of attractables present in the current level.
	void RemoveAttractable(AActor* actor)
	{ if (Attractables.Contains(actor) == true) { Attractables.Remove(actor); Attractables.Compact(); } }

	// Determine the vehicles that are currently present in the level.
	void DetermineVehicles();

	// Determine the pursuit splines that are currently present in the level.
	void DeterminePursuitSplines();

	// Record an event that has just occurred within the game.
	void AddGameEvent(FGameEvent& gameEvent);

	// Convert a master racing spline distance to a lap distance.
	float MasterRacingSplineDistanceToLapDistance(float distance);

	// Determine the master racing spline.
	static UPursuitSplineComponent* DetermineMasterRacingSpline(const FName& navigationLayer, UWorld* world, UGlobalGameState* gameState);

	// Build all of the pursuit splines.
	static void BuildPursuitSplines(bool check, const FName& navigationLayer, UWorld* world, UGlobalGameState* gameState, UPursuitSplineComponent* masterRacingSpline);

	// Establish all of the links between pursuit splines.
	static void EstablishPursuitSplineLinks(bool check, const FName& navigationLayer, UWorld* world, UGlobalGameState* gameState, UPursuitSplineComponent* masterRacingSpline);

	// List of the last few frame times, used to determine an average, recent frame rate.
	FTimedFloatList FrameTimes = FTimedFloatList(1, 30);

	// Get the play game mode for the current world.
	static APlayGameMode* Get(const UObject* worldContextObject)
	{ return (worldContextObject == nullptr) ? nullptr : Cast<APlayGameMode>(UGameplayStatics::GetGameMode(worldContextObject)); }

	// The master racing spline used to calculate race distance.
	// This must be a looped spline.
	TWeakObjectPtr<UPursuitSplineComponent> MasterRacingSpline;

	// The length of the master racing spline in centimeters.
	float MasterRacingSplineLength = 0.0f;

	// The distance around the master racing spline of the start line.
	float MasterRacingSplineStartDistance = 0.0f;

	// The ratio around the last lap, between 0 and 1, 0 being before or at the beginning of the lap and 1 meaning the lap is complete.
	// This is normally used to scale back the aggression of the bots during the last lap to give you a better chance of winning.
	float LastLapRatio = 0.0f;

	// The last time the options were set within the game.
	// This is used to optimize the HUD rendering.
	float LastOptionsResetTime = 0.0f;

	// Not playing from a standard player start in the Editor.
	bool UnknownPlayerStart = false;

	// The time in seconds that the outro should last at the end of a game.
	static const int32 RaceOutroTime = 5;

protected:

	// Do some post initialization just before the game is ready to play.
	virtual void PostInitializeComponents() override;

	// Do some initialization when the game is ready to play.
	virtual void BeginPlay() override;

	// Do some shutdown when the actor is being destroyed.
	virtual void EndPlay(const EEndPlayReason::Type endPlayReason) override;

	// Do the regular update tick, post update work for this actor.
	virtual void Tick(float deltaSeconds) override;

	// Set the graphics options into the system.
	virtual void SetGraphicsOptions(bool initialization) override;

private:

	// Update the race start line stuff, mostly the camera at this point.
	void UpdateRaceStartLine();

	// Calculate the race positions for each of the vehicles.
	void UpdateRacePositions(float deltaSeconds);

	// Upload the loading of the main UI.
	void UpdateUILoading();

	// Calculate the maximum number of players.
	int32 CalculateMaxPlayers() const;

	// Get the local player's vehicle.
	ABaseVehicle* GetPlayerVehicle(int32 localPlayerIndex) const;

	// Quick function for grabbing the children of a panel.
	static void GetAllWidgetsForParent(TArray<UWidget*>& widgets, UPanelWidget* panel);

	// The vehicle index that focus on for player 0, or -1 for none and just focus on your own vehicle.
	int32 VehicleCameraIndex = -1;

	// The number of players destroyed during this game.
	int32 NumPlayersDestroyed = 0;

	// The clock at the time the game finished.
	float GameFinishedAt = 0.0f;

	// A timer used for managing the Elimination game mode.
	float EliminationTimer = -GRIP_ELIMINATION_SECONDS;

	// Which part of the game sequence is the game currently in?
	EGameSequence GameSequence = EGameSequence::None;

	// The next race position for a finishing player.
	int32 NextFinishingRacePosition = 0;

	// The next position for a dead player.
	int32 NextDeathPosition = 0;

	// Has the start camera dropped yet?
	bool StartCameraDropped = false;

	// Variables used to manage the pre-game countdown sequence.
	int32 StartLineDropTime = 0;
	int32 StartLineCountFrom = 0;
	int32 StartLineCountTo = 0;

	// A list of vehicles currently being watched directly by a camera.
	// This is used to help calculate the relative volume level of each of the vehicles effectively.
	TArray<ABaseVehicle*> WatchedVehicles;

	// The pawn that is currently the focus of the camera cycling system.
	UPROPERTY(Transient)
		APawn* ViewingPawn = nullptr;

	// The player starts contained in the current level.
	UPROPERTY(Transient)
		TArray<APlayerStart*> Startpoints;

	// The player starts contained in the current level that have not yet been assigned.
	UPROPERTY(Transient)
		TArray<APlayerStart*> UnusedStartpoints;

	// The frictional actors contained in the current level.
	UPROPERTY(Transient)
		TArray<AActor*> FrictionalActors;

	// The type of widget to use for the single screen UI.
	static TSubclassOf<USingleHUDWidget> SingleScreenWidgetClass;
};
