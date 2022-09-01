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

#include "gamemodes/basegamemode.h"
#include "game/globalgamestate.h"
#include "kismet/kismetmateriallibrary.h"
#include "blueprint/userwidget.h"
#include "blueprint/widgetblueprintlibrary.h"
#include "physicsengine/physicssettings.h"
#include "system/worldfilter.h"
#include "audiodevice.h"

/**
* Mapping of trace channels to the project settings.
***********************************************************************************/

ECollisionChannel ABaseGameMode::ECC_Missile;
ECollisionChannel ABaseGameMode::ECC_VehicleCamera;
ECollisionChannel ABaseGameMode::ECC_VehicleSpring;
ECollisionChannel ABaseGameMode::ECC_LineOfSightTest;
ECollisionChannel ABaseGameMode::ECC_LineOfSightTestIncVehicles;
ECollisionChannel ABaseGameMode::ECC_TerrainFollowing;

/**
* Construct the base game mode.
***********************************************************************************/

ABaseGameMode::ABaseGameMode()
{
	// We need all of the players to be ticked before the game state so that we can
	// calculate race position effectively.

	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PostUpdateWork;
}

/**
* Do some post initialization just before the game is ready to play.
***********************************************************************************/

void ABaseGameMode::PostInitializeComponents()
{
	UE_LOG(GripLog, Log, TEXT("ABaseGameMode::PostInitializeComponents"));

	Super::PostInitializeComponents();

	FSoftClassPath gameInstanceClassName = GetDefault<UGameMapsSettings>()->GameInstanceClass;

	verifyf(gameInstanceClassName.ToString().EndsWith(TEXT(".GlobalGameState")) == true, TEXT("You must set the Game Instance Class in the Maps And Modes section of your Project Settings to GlobalGameState"));

	GlobalGameState = UGlobalGameState::GetGlobalGameState(this);

	UGlobalGameState::LoadGlobalGameState(GetGameInstance());

	FString worldName = GetWorld()->GetPathName();

	UGlobalGameState::SynchronizeInputMapping(GetWorld());

#if GRIP_UNLIMITED_TEXTURE_POOL
	IConsoleManager::Get().FindConsoleVariable(TEXT("r.Streaming.PoolSize"))->Set(0);
#endif // GRIP_UNLIMITED_TEXTURE_POOL
}

/**
* Get the display name text for a collision channel, ripped from Collision.cpp
***********************************************************************************/

static FString GetChannelDisplayNameText(const UEnum* enumeration, int64 value, const FString& fallback)
{
	if (enumeration != nullptr)
	{
#if WITH_EDITOR
		return enumeration->GetDisplayNameTextByValue(value).ToString();
#else
		return FName::NameToDisplayString(enumeration->GetNameStringByValue(value), false);
#endif
	}

	return fallback;
}

/**
* Convert a string to a collision channel, ripped from Collision.cpp
***********************************************************************************/

static ECollisionChannel StringToCollisionChannel(const FString& channelString)
{
	const UEnum* channelEnum = StaticEnum<ECollisionChannel>();

	if (channelEnum != nullptr)
	{
		// Try the name they gave us.

		int32 channelInt = channelEnum->GetValueByName(FName(*channelString));

		if (channelInt != INDEX_NONE)
		{
			return (ECollisionChannel)channelInt;
		}

		// Try with adding the prefix, ie "ECC_".

		const FString withPrefixName = channelEnum->GenerateEnumPrefix() + TEXT("_") + channelString;

		channelInt = channelEnum->GetValueByName(FName(*withPrefixName));

		if (channelInt != INDEX_NONE)
		{
			return (ECollisionChannel)channelInt;
		}

		// Try matching the display name.

		const FString nullString(TEXT(""));

		for (int32 channelIndex = 0; channelIndex < channelEnum->NumEnums(); channelIndex++)
		{
			int64 channelValue = channelEnum->GetValueByIndex(channelIndex);
			const FString channelDisplayName = GetChannelDisplayNameText(channelEnum, channelValue, nullString);

			if (channelDisplayName == channelString)
			{
				return (ECollisionChannel)channelValue;
			}
		}
	}

	// Try parsing a digit, as in from the ListChannels command.

	const int32 channelValue = FCString::IsNumeric(*channelString) ? FCString::Atoi(*channelString) : INDEX_NONE;

	return (channelValue >= 0 && channelValue < ECollisionChannel::ECC_MAX) ? (ECollisionChannel)channelValue : ECollisionChannel::ECC_MAX;
}

/**
* Do some initialization when the game is ready to play.
***********************************************************************************/

void ABaseGameMode::BeginPlay()
{
	UE_LOG(GripLog, Log, TEXT("ABaseGameMode::BeginPlay"));

	SetGraphicsOptions(true);
	SetAudioOptions(true);
	SetInputOptions(true);

	Super::BeginPlay();

#if GRIP_ENGINE_EXTENDED_MODIFICATIONS
	if (FAudioDeviceHandle audioDevice = GetWorld()->GetAudioDevice())
	{
		audioDevice->SetMirroredAudio(GlobalGameState->IsTrackMirrored());
	}
#endif // GRIP_ENGINE_EXTENDED_MODIFICATIONS

	APlayerController* controller = UGameplayStatics::GetPlayerController(this, 0);

	if (controller != nullptr)
	{
		EnableInput(controller);
	}

	Clock = 0.0f;
	RealTimeGameClockTicking = false;
	RealTimeGameClockPausedTime = 0.0;
	RealTimeClock = RealTimeClockStart = RealTimeGameClockStart = FApp::GetCurrentTime();

	PostProcessVolumes.Empty();

	for (TActorIterator<APostProcessVolume> actorItr(GetWorld()); actorItr; ++actorItr)
	{
		APostProcessVolume* postProcess = *actorItr;

		if (FWorldFilter::IsValid(postProcess, GlobalGameState) == true)
		{
			postProcess->Settings.bOverride_FilmSaturation = false;
			postProcess->Settings.FilmSaturation = 1.0f;

			PostProcessVolumes.Emplace(postProcess);
		}
	}

	// Determine all of the collision channels that we need based on their names.

	ECC_Missile = StringToCollisionChannel(TEXT("Missile"));
	ECC_VehicleCamera = StringToCollisionChannel(TEXT("VehicleCamera"));
	ECC_VehicleSpring = StringToCollisionChannel(TEXT("VehicleSpring"));
	ECC_LineOfSightTest = StringToCollisionChannel(TEXT("LineOfSightTest"));
	ECC_LineOfSightTestIncVehicles = StringToCollisionChannel(TEXT("LineOfSightTestIncVehicles"));
	ECC_TerrainFollowing = StringToCollisionChannel(TEXT("TerrainFollowing"));

	UPhysicsSettings* physicsSettings = UPhysicsSettings::Get();

	float fps = 200.0f;

#if GRIP_FIXED_TIMING
	fps = GRIP_TIMING_FPS * GRIP_PHYSICS_SUBSTEPS * 0.95f;
#endif

	physicsSettings->bSubstepping = true;
	physicsSettings->MaxSubstepDeltaTime = 1.0f / fps;

	// We only guarantee the physics to step correctly at a rendering rate of 20Hz or more.
	// After that, the physics will start to slow but the machine is already struggling anyway
	// and we don't want to overburden it with a large number of physics iterations.

	physicsSettings->MaxSubsteps = FMath::CeilToInt((1.0f / physicsSettings->MaxSubstepDeltaTime) / 20.0f);
}

/**
* Do the regular update tick.
***********************************************************************************/

void ABaseGameMode::Tick(float deltaSeconds)
{
	Super::Tick(deltaSeconds);

	FrameNumber++;

	UpdateTimeDilation(deltaSeconds);

	if (Clock == 0.0f)
	{
		// We can assume the first tick won't be time-dilated here.

		RealTimeClockStart = RealTimeGameClockStart = FApp::GetCurrentTime() - deltaSeconds;
	}

	Clock += deltaSeconds;

	// More accurate to do the math this way for the real-time clock.

	RealTimeClock = FApp::GetCurrentTime();

	if (RealTimeGameClockTicking == false)
	{
		RealTimeGameClockStart = RealTimeClock;
	}
	else if (RealTimeGameClockPausedTime != 0.0 && UGameplayStatics::IsGamePaused(this) == false)
	{
		RealTimeGameClockStart += RealTimeClock - RealTimeGameClockPausedTime;
		RealTimeGameClockPausedTime = 0.0;
	}
}

/**
* Set the graphics options into the system.
***********************************************************************************/

void ABaseGameMode::SetGraphicsOptions(bool initialization)
{
	FGraphicsOptions& graphics = GlobalGameState->GraphicsOptions;
	FScreenResolution& resolution = graphics.ScreenResolution;
	EScreenMode mode = GlobalGameState->GraphicsOptions.ScreenMode;
	bool vsync = graphics.ScreenSync == EOffOnSwitch::On;
	UGameUserSettings* settings = GEngine->GetGameUserSettings();

	GlobalGameState->SetupDefaultResolution();

	FIntPoint oldResolution = settings->GetScreenResolution();

	if (settings->SupportsHDRDisplayOutput() == true &&
		GlobalGameState->GraphicsOptions.HighDynamicRange == EOffOnSwitch::On)
	{
		mode = EScreenMode::FullScreen;
	}

	settings->SetVSyncEnabled(vsync);

	if (initialization == true)
	{
		switch (mode)
		{
		default:
			settings->SetFullscreenMode(EWindowMode::Fullscreen);
			break;
		case EScreenMode::FramedWindow:
			settings->SetFullscreenMode(EWindowMode::Windowed);
			break;
		case EScreenMode::FullScreenWindow:
			settings->SetFullscreenMode(EWindowMode::WindowedFullscreen);
			break;
		}
	}

	settings->SetScreenResolution(FIntPoint(resolution.Width, resolution.Height));

	static IConsoleVariable* antiAliasing = IConsoleManager::Get().FindConsoleVariable(TEXT("r.DefaultFeature.AntiAliasing"));

	if (antiAliasing != nullptr)
	{
		switch (GlobalGameState->GraphicsOptions.AntiAliasing)
		{
		case EAntiAliasingMode::None:
			antiAliasing->Set(0);
			break;
		case EAntiAliasingMode::FXAA:
			antiAliasing->Set(1);
			break;
		case EAntiAliasingMode::TemporalAA:
			antiAliasing->Set(2);
			break;
		}
	}

	Scalability::FQualityLevels levels = settings->ScalabilityQuality;

	// 0 : low, 1 : medium, 2 : high, 3 : epic

	levels.ViewDistanceQuality = FMath::Clamp((int32)graphics.ObjectsDetail - 1, 0, 3);
	levels.AntiAliasingQuality = FMath::Clamp((int32)graphics.AntiAliasingDetail - 1, 0, 3);
	levels.ShadowQuality = FMath::Clamp((int32)graphics.ShadowDetail - 1, 0, 3);
	levels.PostProcessQuality = FMath::Clamp((int32)graphics.PostProcessingDetail - 1, 0, 3);
	levels.TextureQuality = FMath::Clamp((int32)graphics.TextureDetail - 1, 0, 3);
	levels.EffectsQuality = FMath::Clamp((int32)graphics.EffectsDetail - 1, 0, 3);
	levels.ResolutionQuality = FMath::Clamp(graphics.ResolutionQuality, 25, 200);

	settings->ScalabilityQuality = levels;

	if (initialization == false)
	{
		// If within the app settings panel, then set the display mode and save it before restoring it.
		// We do this because we can't change the screen mode in the middle of the app, but we do
		// want it saved.

		EWindowMode::Type oldMode = settings->GetFullscreenMode();

		switch (mode)
		{
		default:
			settings->SetFullscreenMode(EWindowMode::Fullscreen);
			break;
		case EScreenMode::FramedWindow:
			settings->SetFullscreenMode(EWindowMode::Windowed);
			break;
		case EScreenMode::FullScreenWindow:
			settings->SetFullscreenMode(EWindowMode::WindowedFullscreen);
			break;
		}

		settings->SaveSettings();

		settings->SetFullscreenMode(oldMode);
	}
	else
	{
		settings->SaveSettings();
	}

	if (settings->SupportsHDRDisplayOutput() == true)
	{
		UE_LOG(GripLog, Log, TEXT("HDR on this PC, setting to %d %d"), GlobalGameState->GraphicsOptions.HighDynamicRange == EOffOnSwitch::On ? 1 : 0, GlobalGameState->GraphicsOptions.DisplayNits);

		settings->EnableHDRDisplayOutput(GlobalGameState->GraphicsOptions.HighDynamicRange == EOffOnSwitch::On, GlobalGameState->GraphicsOptions.DisplayNits);
	}
	else
	{
		UE_LOG(GripLog, Log, TEXT("No HDR on this PC"));

		settings->EnableHDRDisplayOutput(false);
	}

#if GRIP_FIXED_TIMING
	settings->SetFrameRateLimit(GRIP_TIMING_FPS);
#else // GRIP_FIXED_TIMING
	if (graphics.LimitMaxFPS == EOffOnSwitch::On)
	{
		settings->SetFrameRateLimit(0);

		GEngine->bUseFixedFrameRate = false;
		GEngine->bSmoothFrameRate = true;
		GEngine->SmoothedFrameRateRange = FFloatRange(FMath::Max(20.0f, GEngine->SmoothedFrameRateRange.GetLowerBoundValue()), graphics.MaxFPS);
	}
	else
	{
		settings->SetFrameRateLimit(0);

		GEngine->bUseFixedFrameRate = false;
		GEngine->bSmoothFrameRate = true;
		GEngine->SmoothedFrameRateRange = FFloatRange(FMath::Max(20.0f, GEngine->SmoothedFrameRateRange.GetLowerBoundValue()), FMath::Max(250.0f, GEngine->SmoothedFrameRateRange.GetUpperBoundValue()));
	}
#endif // GRIP_FIXED_TIMING

	settings->ConfirmVideoMode();

	if (initialization == true &&
		oldResolution != FIntPoint(resolution.Width, resolution.Height))
	{
		settings->ApplySettings(true);
	}
	else
	{
		settings->ApplyNonResolutionSettings();
	}
}

/**
* Set the audio options into the system.
***********************************************************************************/

void ABaseGameMode::SetAudioOptions(bool initialization)
{
	SetAudioOptionsForVolume();
}

/**
* Set the input options into the system.
***********************************************************************************/

void ABaseGameMode::SetInputOptions(bool initialization)
{
	if (initialization == false)
	{
		for (TActorIterator<APlayerController> actorItr(GetWorld()); actorItr; ++actorItr)
		{
			SetInputOptions(*actorItr);
		}
	}
}

/**
* Set the input options into a controller.
***********************************************************************************/

void ABaseGameMode::SetInputOptions(APlayerController* controller)
{
	if (GRIP_OBJECT_VALID(controller) == true &&
		controller->IsLocalPlayerController() == true)
	{
		ULocalPlayer* localPlayer = controller->GetLocalPlayer();

		if (localPlayer != nullptr)
		{
			UGlobalGameState* gameState = UGlobalGameState::GetGlobalGameState(GetWorld());
			int32 vehicleIndex = localPlayer->GetControllerId();

			if (gameState != nullptr &&
				gameState->InputControllerOptions.IsValidIndex(vehicleIndex) == true)
			{
				controller->PlayerInput->ActionMappings.Empty();
				controller->PlayerInput->AxisMappings.Empty();

				for (FInputActionKeyMapping& mapping : gameState->InputControllerOptions[vehicleIndex].ActionMappings)
				{
					if (mapping.Key != EKeys::Invalid)
					{
						controller->PlayerInput->ActionMappings.Emplace(mapping);
					}
				}

				for (FInputAxisKeyMapping& mapping : gameState->InputControllerOptions[vehicleIndex].AxisMappings)
				{
					if (mapping.Key != EKeys::Invalid)
					{
						controller->PlayerInput->AxisMappings.Emplace(mapping);
					}
				}

				controller->PlayerInput->ForceRebuildingKeyMaps(false);
			}
		}
	}
}

/**
* Load a map after showing the loading screen.
***********************************************************************************/

void ABaseGameMode::LoadTrack()
{
	UE_LOG(GripLog, Log, TEXT("ABaseGameMode::LoadTrack"));

	GlobalGameState->TransientGameState.NavigationLayer = TEXT("ForwardNavigation");

	LoadMap(GlobalGameState->TransientGameState.MapName, false);
}

/**
* Load a map after showing the loading screen.
***********************************************************************************/

void ABaseGameMode::LoadMap(const FString& mapName, bool menuMode)
{
	if (mapName.Len() > 0 &&
		mapName != "None")
	{
		UE_LOG(GripLog, Log, TEXT("ABaseGameMode::LoadMap %s"), *mapName);

		APlayerController* controller = UGameplayStatics::GetPlayerController(this, 0);

		// Determine how many local players we need based on the split-screen setup.

		if (GlobalGameState->GamePlaySetup.GameType == EGameType::SplitScreenEvent)
		{
			// This is set in the menu, ensure it's a value between 2 and 4.

			GlobalGameState->TransientGameState.NumberOfLocalPlayers = FMath::Clamp(GlobalGameState->TransientGameState.NumberOfLocalPlayers, 2, GlobalGameState->GetMaxLocalPlayers());
		}
		else
		{
			GlobalGameState->TransientGameState.NumberOfLocalPlayers = 1;
		}

		TSharedPtr<GenericApplication> PlatformApplication = FSlateApplicationBase::Get().GetPlatformApplication();

		if (mapName.Contains("UserInterface") == true)
		{
			// We're loading the main menu.

			UWidgetBlueprintLibrary::SetInputMode_GameAndUIEx(controller);

			if (controller != nullptr)
			{
				controller->bShowMouseCursor = true;

				if (PlatformApplication.IsValid() &&
					PlatformApplication->Cursor.IsValid())
				{
					PlatformApplication->Cursor->Show(true);
				}
			}
		}
		else
		{
			// We're loading a game level.

			GlobalGameState->TransientGameState.EventQuit = false;

			// Player 0 will be created automatically by the engine, so we just need
			// to create any further players for split-screen mode.

			// NOTE: We need to unintuitively do this here, because doing it once the
			// map is loaded is hit and miss whether it will work or not. So, calling
			// CreatePlayer with spawnPlayer == false solves this problem, as that
			// pawn will be created by the engine not now, but as the map loads.

			for (int32 i = 0; i < GlobalGameState->TransientGameState.NumberOfLocalPlayers; i++)
			{
				UGameplayStatics::CreatePlayer(this, i, false);
			}

			UWidgetBlueprintLibrary::SetInputMode_GameOnly(controller);

			if (controller != nullptr)
			{
				controller->bShowMouseCursor = false;

				if (PlatformApplication.IsValid() &&
					PlatformApplication->Cursor.IsValid())
				{
					PlatformApplication->Cursor->Show(false);
				}
			}
		}

		UGameMapsSettings* gameMapsSettings = Cast<UGameMapsSettings>(UGameMapsSettings::StaticClass()->GetDefaultObject());

		if (GlobalGameState->GeneralOptions.SplitScreenLayout == ESplitScreenLayout::TwoPlayerVertical)
		{
			gameMapsSettings->TwoPlayerSplitscreenLayout = ETwoPlayerSplitScreenType::Vertical;
		}
		else
		{
			gameMapsSettings->TwoPlayerSplitscreenLayout = ETwoPlayerSplitScreenType::Horizontal;
		}

		ResetPlayerStarts = true;

		GetWorld()->ServerTravel(mapName);
	}
}

/**
* Shorten some text.
***********************************************************************************/

FString ABaseGameMode::ShortenString(const FString& text, int32 maxLength)
{
	if (text.Len() > maxLength)
	{
		for (int32 i = text.Len() - 1; i > 0; i--)
		{
			if (text[i] == ' ' ||
				text[i] == '-')
			{
				if (text.Len() - i < (maxLength >> 2))
				{
					return text.Left(i);
				}
			}
		}

		FString newText = text.Left(maxLength - 3);

		newText += TEXT("...");

		return newText;
	}

	return text;
}

/**
* Set the time dilation to a new value taking a duration in seconds to transition to
* that new value.
***********************************************************************************/

void ABaseGameMode::ChangeTimeDilation(float slomoRelativeSpeed, float transitionDuration)
{
	if (GlobalGameState->UsingSplitScreen() == false)
	{
		TimeDilationRelativeSpeedTarget = slomoRelativeSpeed;
		TimeDilationTransitionDuration = transitionDuration / FMath::Abs(TimeDilationRelativeSpeed - TimeDilationRelativeSpeedTarget);

		if (transitionDuration > 0.0f)
		{
			TimeDilationDirection = (TimeDilationRelativeSpeedTarget - TimeDilationRelativeSpeed);
		}
		else
		{
			TimeDilationDirection = 0.0f;
			TimeDilationRelativeSpeed = TimeDilationRelativeSpeedTarget;

			UGameplayStatics::SetGlobalTimeDilation(this, FMath::Max(TimeDilationRelativeSpeed, 0.01f));
		}
	}
}

/**
* Update the slow motion ease in and out.
***********************************************************************************/

void ABaseGameMode::UpdateTimeDilation(float deltaSeconds)
{
	float slomoRelativeSpeed = TimeDilationRelativeSpeedTarget;

	if (TimeDilationRelativeSpeed != slomoRelativeSpeed)
	{
		if (TimeDilationDirection != 0.0f)
		{
			float elapsedDeltaSeconds = deltaSeconds / UGameplayStatics::GetGlobalTimeDilation(this);

			TimeDilationRelativeSpeed += (elapsedDeltaSeconds / TimeDilationTransitionDuration) * TimeDilationDirection;

			if (TimeDilationDirection > 0.0f)
			{
				// Speeding up.

				TimeDilationRelativeSpeed = FMath::Min(TimeDilationRelativeSpeed, slomoRelativeSpeed);
			}
			else
			{
				// Slowing down.

				TimeDilationRelativeSpeed = FMath::Max(TimeDilationRelativeSpeed, slomoRelativeSpeed);
			}
		}
		else
		{
			TimeDilationRelativeSpeed = slomoRelativeSpeed;
		}

		float timeScale = FMath::Max(TimeDilationRelativeSpeed, 0.01f);

		UGameplayStatics::SetGlobalTimeDilation(this, timeScale);
	}
}

/**
* Get the size of the viewport used for rendering the game.
***********************************************************************************/

void ABaseGameMode::GetGameViewportSize(int32& x, int32& y, APlayerController* controller)
{
	x = y = 0;

	FVector2D result = FVector2D(0, 0);

	if (controller != nullptr)
	{
		controller->GetViewportSize(x, y);
	}

	if (x == 0)
	{
		if (GEngine && GEngine->GameViewport)
		{
			GEngine->GameViewport->GetViewportSize(result);
		}

		if (result.X == 0.0f)
		{
			result.X = GSystemResolution.ResX;
			result.Y = GSystemResolution.ResY;
		}

		x = result.X;
		y = result.Y;
	}
}

/**
* Get the color used to signify a particular team number.
***********************************************************************************/

FLinearColor ABaseGameMode::GetTeamColour(int32 teamNumber, float opacity)
{
	switch (teamNumber)
	{
	case 0:
		return FLinearColor(1.0f, 0.0f, 0.0f, opacity);
	case 1:
		return FLinearColor(0.0322f, 0.275f, 1.0f, opacity);
	default:
		return FLinearColor(1.0f, 1.0f, 1.0f, opacity);
	}
}

/**
* Get the name of a player.
***********************************************************************************/

FString ABaseGameMode::GetPlayerName(class APlayerState* player, int32 playerNumber, bool upperCase, bool forceGeneric)
{
	if (forceGeneric == true
#if GRIP_GENERIC_PLAYER_NAME
		|| true)
#else // GRIP_GENERIC_PLAYER_NAME
		)
#endif // GRIP_GENERIC_PLAYER_NAME
	{
		FFormatNamedArguments arguments; arguments.Add(TEXT("PlayerIndex"), FText::AsNumber(playerNumber));

		return FText::Format(NSLOCTEXT("Grip", "PlayerNumber", "PLAYER {PlayerIndex}"), arguments).ToString();
	}
	else
	{
		if (upperCase == true)
		{
			return player->GetPlayerName().ToUpper();
		}
		else
		{
			return player->GetPlayerName();
		}
	}
}

/**
* Sleep a component so that it's invisible and no longer having its transform
* hierarchy updated.
***********************************************************************************/

void ABaseGameMode::SleepComponent(USceneComponent* component)
{
	if (component != nullptr &&
		component->IsUsingAbsoluteLocation() == false)
	{
		component->SetUsingAbsoluteLocation(true);
		component->SetUsingAbsoluteRotation(true);
		component->SetUsingAbsoluteScale(true);
		component->SetHiddenInGame(true);
	}
}

/**
* Wake a component so that it's visible and has its transform hierarchy updated.
***********************************************************************************/

void ABaseGameMode::WakeComponent(USceneComponent* component)
{
	if (component != nullptr &&
		component->IsUsingAbsoluteLocation() == true)
	{
		component->SetUsingAbsoluteLocation(false);
		component->SetUsingAbsoluteRotation(false);
		component->SetUsingAbsoluteScale(false);
		component->UpdateChildTransforms();
		component->SetHiddenInGame(false);
	}
}
