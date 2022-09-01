/**
*
* Base game mode implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* The base game mode to use for the game. It's inherited by the AMenuGameMode
* and APlayGameMode classes, for menu and playing events respectively.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "gameframework/gamemode.h"
#include "game/globalgamestate.h"
#include "basegamemode.generated.h"

/**
* The base game mode to use for the game. It's inherited by the AMenuGameMode
* and APlayGameMode classes, for menu and playing events respectively.
***********************************************************************************/

UCLASS(Abstract, NotBlueprintable)
class GRIP_API ABaseGameMode : public AGameMode
{
	GENERATED_BODY()

public:

	// Construct the base game mode.
	ABaseGameMode();

	// Setup the game audio for the current volume levels.
	UFUNCTION(BlueprintImplementableEvent, BlueprintCallable, Category = System)
		void SetAudioOptionsForVolume();

	// Load a map after showing the loading screen.
	UFUNCTION(BlueprintCallable, Category = "System")
		virtual void LoadMap(const FString& mapName, bool menuMode = false);

	// Load a map after showing the loading screen.
	UFUNCTION(BlueprintCallable, Category = "System")
		void LoadTrack();

	// Set the graphics options into the system.
	UFUNCTION(BlueprintCallable, Category = "System")
		virtual void SetGraphicsOptions(bool initialization);

	// Set the audio options into the system.
	UFUNCTION(BlueprintCallable, Category = "System")
		virtual void SetAudioOptions(bool initialization);

	// Set the input options into the system.
	UFUNCTION(BlueprintCallable, Category = "System")
		virtual void SetInputOptions(bool initialization);

	// Shorten some text.
	UFUNCTION(BlueprintCallable, Category = "System")
		static FString ShortenString(const FString& text, int32 maxLength);

	// Get the color used to signify a particular team number.
	UFUNCTION(BlueprintCallable, Category = Default)
		static FLinearColor GetTeamColour(int32 teamNumber = -1, float opacity = 1.0f);

	// Get the amount of perceptual time passed since the game mode began, this follows any time dilation used for slow motion.
	UFUNCTION(BlueprintCallable, Category = System)
		float GetClock() const
	{ return Clock; }

	// Get the amount of real time passed since the game mode began.
	UFUNCTION(BlueprintCallable, Category = System)
		float GetRealTimeClock() const
	{ return (float)(RealTimeClock - RealTimeClockStart); }

	// Get the amount of real time passed since the game event began, this won't advance when the game is paused,
	// and doesn't advance until the game actually begins.
	float GetRealTimeGameClock() const
	{ return (float)(RealTimeClock - RealTimeGameClockStart); }

	// Set the input options into a controller.
	void SetInputOptions(APlayerController* controller);

	// Set the time dilation to a new value taking a duration in seconds to transition to that new value.
	void ChangeTimeDilation(float slomoRelativeSpeed, float transitionDuration);

	// Are we currently using time dilation?
	bool IsUsingTimeDilation()
	{ return (TimeDilationRelativeSpeedTarget != 1.0f || TimeDilationRelativeSpeed != 1.0f); }

	// Get the size of the viewport used for rendering the game.
	static void GetGameViewportSize(int32& x, int32& y, APlayerController* controller = nullptr);

	// Sleep a component so that it's invisible and no longer having its transform hierarchy updated.
	static void SleepComponent(USceneComponent* component);

	// Wake a component so that it's visible and has its transform hierarchy updated.
	static void WakeComponent(USceneComponent* component);

	// Get the name of a player.
	static FString GetPlayerName(class APlayerState* player, int32 playerNumber, bool upperCase = false, bool forceGeneric = false);

	// Get the base game mode for the current world.
	static ABaseGameMode* Get(const UObject* worldContextObject)
	{ return (worldContextObject == nullptr) ? nullptr : Cast<ABaseGameMode>(UGameplayStatics::GetGameMode(worldContextObject)); }

	// Mapping of collision channels to the project settings.
	static ECollisionChannel ECC_Missile;
	static ECollisionChannel ECC_VehicleCamera;
	static ECollisionChannel ECC_VehicleSpring;
	static ECollisionChannel ECC_LineOfSightTest;
	static ECollisionChannel ECC_LineOfSightTestIncVehicles;
	static ECollisionChannel ECC_TerrainFollowing;

protected:

	// Do some post initialization just before the game is ready to play.
	virtual void PostInitializeComponents() override;

	// Do some initialization when the game is ready to play.
	virtual void BeginPlay() override;

	// Do the regular update tick.
	virtual void Tick(float deltaSeconds) override;

	// Update the slow motion ease in and out.
	virtual void UpdateTimeDilation(float deltaSeconds);

	// Reset the player starts at the beginning of a game mode.
	bool ResetPlayerStarts = true;

	// The frame number since the game mode was started.
	int32 FrameNumber = 0;

	// The direction in which the slow motion is heading in, -1 for slower and +1 for faster.
	float TimeDilationDirection = 0.0f;

	// The current slow motion relative speed, 1.0 for normal speed.
	float TimeDilationRelativeSpeed = 1.0f;

	// The target slow motion relative speed, 1.0 for normal speed.
	float TimeDilationRelativeSpeedTarget = 1.0f;

	// The duration over which to adjust the slow motion relative speed to its target.
	float TimeDilationTransitionDuration = 1.0f;

	// The amount of perceptual time passed since the game mode began, this follows any time dilation used for slow motion.
	float Clock = 0.0f;

	// The real-time clock, taken from FApp::GetCurrentTime().
	double RealTimeClock = 0.0;

	// The real-time clock at the point of the first frame of the current game mode, taken from FApp::GetCurrentTime().
	double RealTimeClockStart = 0.0;

	// The real-time clock at the point of the first frame of the current game mode, taken from FApp::GetCurrentTime().
	double RealTimeGameClockStart = 0.0;

	// The real-time clock at the point the game was last paused, taken from FApp::GetCurrentTime().
	double RealTimeGameClockPausedTime = 0.0;

	// Is the real-time game clock ticking yet? It won't be until the event begins, normally a few seconds into the game mode.
	bool RealTimeGameClockTicking = false;

	// The post-process volumes found in the current level, stored here for fast iteration.
	UPROPERTY(Transient)
		TArray<APostProcessVolume*> PostProcessVolumes;

	// Naked pointer to game state for performance reasons.
	UPROPERTY(Transient)
		UGlobalGameState* GlobalGameState = nullptr;
};
