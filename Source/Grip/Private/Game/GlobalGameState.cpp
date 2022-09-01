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

#include "game/globalgamestate.h"
#include "gamemodes/playgamemode.h"
#include "sound/soundclass.h"

/**
* Construct a global game state.
***********************************************************************************/

UGlobalGameState::UGlobalGameState()
{
	VerifyInputOptions(true);
}

/**
* Is this a shipping build of the game?
***********************************************************************************/

bool UGlobalGameState::IsShippingBuild()
#if UE_BUILD_SHIPPING
{
	return true;
}
#else // UE_BUILD_SHIPPING
{
	return false;
}
#endif // UE_BUILD_SHIPPING

/**
* Get the power scale of the engines used by vehicles.
***********************************************************************************/

float FGeneralOptions::GetEnginePowerScale(int32 difficultyLevel, int32 enginePowerLevel) const
{
	if (enginePowerLevel == -1)
	{
		enginePowerLevel = ((EnginePowerLevel == 3) ? difficultyLevel : EnginePowerLevel);
	}

	switch (enginePowerLevel)
	{
	default:
		return 1.0f;
	case 1:
		return 1.15f;
	case 2:
		return 1.3f;
	case 3:
		return 1.3f;
	}
}

/**
* Get the drag scale used by vehicles.
***********************************************************************************/

float FGeneralOptions::GetDragScale(int32 difficultyLevel) const
{
	if (EnginePowerLevel == 0)
	{
		switch (difficultyLevel)
		{
		default:
			return 1.2f;
		case 1:
			return 1.0f;
		case 2:
			return 1.0f;
		case 3:
			return 1.0f;
		}
	}

	return 1.0f;
}

/**
* Synchronize the input with the editor.
***********************************************************************************/

void UGlobalGameState::SynchronizeInputMapping(UWorld* world)
{
	UGlobalGameState* state = GetGlobalGameState(world);

	if (state != nullptr)
	{
		state->VerifyInputOptions(true);

		for (int32 i = 0; i < state->InputControllerOptions.Num(); i++)
		{
			state->ResetPlayerInput(i, true);
		}
	}
}

/**
* Set the volume of a particular sound class.
***********************************************************************************/

void UGlobalGameState::SetSoundClassVolume(USoundClass* soundClass, float volume)
{
	if (soundClass != nullptr)
	{
		soundClass->Properties.Volume = volume;
	}
}

/**
* Reset the input for a player.
***********************************************************************************/

bool UGlobalGameState::ResetPlayerInput(int32 playerID, bool fillGaps)
{
	if (playerID >= 0 &&
		playerID < InputControllerOptions.Num())
	{
		if (fillGaps == false)
		{
			InputControllerOptions[playerID].AxisMappings.Empty();
			InputControllerOptions[playerID].ActionMappings.Empty();
		}

		TArray<FName> addedActions;

		if (playerID == 0)
		{
			// Player 1

			static FInputActionKeyMapping mappings[] =
			{
				FInputActionKeyMapping("Brake", EKeys::SpaceBar, false, false, false, false),
				FInputActionKeyMapping("UsePickup1", EKeys::LeftShift, false, false, false, false),
				FInputActionKeyMapping("UsePickup2", EKeys::LeftControl, false, false, false, false),
				FInputActionKeyMapping("SwitchTarget", EKeys::T, false, false, false, false),
				FInputActionKeyMapping("TeleportToTrack", EKeys::R, false, false, false, false),
				FInputActionKeyMapping("LookLeft", EKeys::V, false, false, false, false),
				FInputActionKeyMapping("LookRight", EKeys::N, false, false, false, false),
				FInputActionKeyMapping("LookBack", EKeys::B, false, false, false, false),
				FInputActionKeyMapping("CameraIn", EKeys::Equals, false, false, false, false),
				FInputActionKeyMapping("CameraOut", EKeys::Hyphen, false, false, false, false),
				FInputActionKeyMapping("LaunchCharge", EKeys::L, false, false, false, false),
				FInputActionKeyMapping("Boost", EKeys::Enter, false, false, false, false),
			};

			for (int32 i = 0; i < sizeof(mappings) / sizeof(mappings[0]); i++)
			{
				bool found = false;

				for (FInputActionKeyMapping& mapping : InputControllerOptions[playerID].ActionMappings)
				{
					if (mapping.ActionName == mappings[i].ActionName)
					{
						found = true;
						break;
					}
				}

				if (found == false)
				{
					addedActions.Emplace(mappings[i].ActionName);
					InputControllerOptions[playerID].ActionMappings.Emplace(mappings[i]);
				}
			}

			static FInputAxisKeyMapping axisMappings[] =
			{
				FInputAxisKeyMapping("Throttle", EKeys::Up, 1.00000f),
				FInputAxisKeyMapping("Throttle", EKeys::Down, -1.00000f),
				FInputAxisKeyMapping("DigitalSteering", EKeys::Left, -1.00000f),
				FInputAxisKeyMapping("DigitalSteering", EKeys::Right, 1.00000f),
				FInputAxisKeyMapping("LookForwards", EKeys::Invalid, 1.00000f),
				FInputAxisKeyMapping("LookSideways", EKeys::Invalid, 1.00000f)
			};

			for (int32 i = 0; i < sizeof(axisMappings) / sizeof(axisMappings[0]); i++)
			{
				bool found = false;

				for (FInputAxisKeyMapping& mapping : InputControllerOptions[playerID].AxisMappings)
				{
					if (mapping.AxisName == axisMappings[i].AxisName &&
						mapping.Scale == axisMappings[i].Scale)
					{
						found = true;
						break;
					}
				}

				if (found == false)
				{
					addedActions.Emplace(axisMappings[i].AxisName);
					InputControllerOptions[playerID].AxisMappings.Emplace(axisMappings[i]);
				}
			}
		}

		// Common

		static FInputActionKeyMapping mappings[] =
		{
			FInputActionKeyMapping("Brake", EKeys::Gamepad_FaceButton_Left, false, false, false, false),
			FInputActionKeyMapping("UsePickup1", EKeys::Gamepad_LeftShoulder, false, false, false, false),
			FInputActionKeyMapping("UsePickup2", EKeys::Gamepad_RightShoulder, false, false, false, false),
			FInputActionKeyMapping("SwitchTarget", EKeys::Gamepad_Special_Left, false, false, false, false),
			FInputActionKeyMapping("TeleportToTrack", EKeys::Gamepad_FaceButton_Right, false, false, false, false),
			FInputActionKeyMapping("LookLeft", EKeys::Invalid, false, false, false, false),
			FInputActionKeyMapping("LookRight", EKeys::Invalid, false, false, false, false),
			FInputActionKeyMapping("LookBack", EKeys::Invalid, false, false, false, false),
			FInputActionKeyMapping("CameraIn", EKeys::Gamepad_DPad_Up, false, false, false, false),
			FInputActionKeyMapping("CameraOut", EKeys::Gamepad_DPad_Down, false, false, false, false),
			FInputActionKeyMapping("LaunchCharge", EKeys::Gamepad_FaceButton_Bottom, false, false, false, false),
			FInputActionKeyMapping("Boost", EKeys::Gamepad_FaceButton_Top, false, false, false, false)
		};

		for (int32 i = 0; i < sizeof(mappings) / sizeof(mappings[0]); i++)
		{
			bool found = false;

			if (addedActions.Contains(mappings[i].ActionName) == false)
			{
				for (FInputActionKeyMapping& mapping : InputControllerOptions[playerID].ActionMappings)
				{
					if (mapping.ActionName == mappings[i].ActionName)
					{
						found = true;
						break;
					}
				}
			}

			if (found == false)
			{
				addedActions.Emplace(mappings[i].ActionName);
				InputControllerOptions[playerID].ActionMappings.Emplace(mappings[i]);
			}
		}

		static FInputAxisKeyMapping axisMappings[] =
		{
			FInputAxisKeyMapping("Throttle", EKeys::Gamepad_RightTriggerAxis, 1.00000f),
			FInputAxisKeyMapping("Throttle", EKeys::Gamepad_LeftTriggerAxis, -1.00000f),
			FInputAxisKeyMapping("DigitalSteering", EKeys::Gamepad_DPad_Left, -1.00000f),
			FInputAxisKeyMapping("DigitalSteering", EKeys::Gamepad_DPad_Right, 1.00000f),
			FInputAxisKeyMapping("AnalogSteering", EKeys::Gamepad_LeftX, 1.00000f),
			FInputAxisKeyMapping("PitchInput", EKeys::Gamepad_LeftY, 1.00000f),
			FInputAxisKeyMapping("LookForwards", EKeys::Gamepad_RightY, 1.00000f),
			FInputAxisKeyMapping("LookSideways", EKeys::Gamepad_RightX, 1.00000f)
		};

		for (int32 i = 0; i < sizeof(axisMappings) / sizeof(axisMappings[0]); i++)
		{
			bool found = false;

			if (addedActions.Contains(axisMappings[i].AxisName) == false)
			{
				for (FInputAxisKeyMapping& mapping : InputControllerOptions[playerID].AxisMappings)
				{
					if (mapping.AxisName == axisMappings[i].AxisName &&
						mapping.Scale == axisMappings[i].Scale)
					{
						found = true;
						break;
					}
				}
			}

			if (found == false)
			{
				InputControllerOptions[playerID].AxisMappings.Emplace(axisMappings[i]);
			}
		}

		// Now remove any duplicates, with the later ones taking priority.

		for (int32 i = InputControllerOptions[playerID].ActionMappings.Num() - 1; i >= 0; i--)
		{
			for (int32 j = i - 1; j >= 0; j--)
			{
				FInputActionKeyMapping& mapping0 = InputControllerOptions[playerID].ActionMappings[i];
				FInputActionKeyMapping& mapping1 = InputControllerOptions[playerID].ActionMappings[j];

				if (mapping1.Key != EKeys::Invalid &&
					mapping0.Key == mapping1.Key)
				{
					InputControllerOptions[playerID].ActionMappings[j].Key = EKeys::Invalid;
				}
			}
		}

		// Check for any non axis right stick bindings.

		InputControllerOptions[playerID].IgnoreRightStick = false;

		for (FInputActionKeyMapping& mapping : InputControllerOptions[playerID].ActionMappings)
		{
			if (mapping.Key == EKeys::Gamepad_RightStick_Down || mapping.Key == EKeys::Gamepad_RightStick_Left ||
				mapping.Key == EKeys::Gamepad_RightStick_Up || mapping.Key == EKeys::Gamepad_RightStick_Right)
			{
				InputControllerOptions[playerID].IgnoreRightStick = true;
			}
		}

		return true;
	}

	return false;
}

/**
* Sanity-check the input options.
***********************************************************************************/

void UGlobalGameState::VerifyInputOptions(bool fillEmptyMappings)
{
	while (InputControllerOptions.Num() < GRIP_MAX_LOCAL_PLAYERS)
	{
		InputControllerOptions.Emplace(FInputControllerOptions());
	}

	if (fillEmptyMappings == true)
	{
		for (int32 i = 0; i < InputControllerOptions.Num(); i++)
		{
			if (InputControllerOptions[i].AxisMappings.Num() == 0 ||
				InputControllerOptions[i].ActionMappings.Num() == 0)
			{
				ResetPlayerInput(i, false);
			}
		}
	}
}

/**
* Setup a default resolution derived from cues from the current hardware.
***********************************************************************************/

void UGlobalGameState::SetupDefaultResolution()
{
	FScreenResolution& resolution = GraphicsOptions.ScreenResolution;

	if (resolution.Width == 0)
	{
		UGameUserSettings* settings = GEngine->GetGameUserSettings();
		FIntPoint desktopResolution = settings->GetDesktopResolution();

		if (desktopResolution.X == 0)
		{
			resolution.Width = 1920;
			resolution.Height = 1080;
		}
		else
		{
			resolution.Width = desktopResolution.X;
			resolution.Height = desktopResolution.Y;
		}
	}
}

/**
* Save the global game state to a file.
***********************************************************************************/

void UGlobalGameState::SaveGlobalGameState(UGameInstance* instance)
{
	UE_LOG(GripLog, Log, TEXT("UGlobalGameState::SaveGlobalGameState"));

	UGlobalGameState* state = UGlobalGameState::GetGlobalGameState(instance, false);

	if (state != nullptr)
	{
		ULocalPlayer* firstPlayer = state->GetFirstGamePlayer();

		if (firstPlayer != nullptr)
		{
			FUniqueNetIdRepl uniqueNetId = firstPlayer->GetPreferredUniqueNetId();

			if (uniqueNetId.IsValid())
			{
				USaveGameSetup* saveGame = Cast<USaveGameSetup>(UGameplayStatics::CreateSaveGameObject(USaveGameSetup::StaticClass()));

				saveGame->PlayerName = firstPlayer->GetNickname();
				saveGame->PlayerId = uniqueNetId->ToString();

				saveGame->GamePlaySetup = state->GamePlaySetup;
				saveGame->GeneralOptions = state->GeneralOptions;
				saveGame->AudioOptions = state->AudioOptions;
				saveGame->GraphicsOptions = state->GraphicsOptions;
				saveGame->InputControllerOptions = state->InputControllerOptions;

				UGameplayStatics::SaveGameToSlot(saveGame, "GlobalGameStateData", firstPlayer->GetControllerId());
			}
			else
			{
				UE_LOG(GripLog, Warning, TEXT("UGlobalGameState::SaveGlobalGameState - no FUniqueNetIdRepl found, abandoning save attempt"));
			}
		}
	}
}

/**
* Load the global game state from a file.
***********************************************************************************/

void UGlobalGameState::LoadGlobalGameState(UGameInstance* instance, bool force)
{
	UGlobalGameState* state = UGlobalGameState::GetGlobalGameState(instance, false);

	if ((state != nullptr) &&
		((state->Loaded == false) || force == true))
	{
		UE_LOG(GripLog, Log, TEXT("UGlobalGameState::LoadGlobalGameState"));

		state->Loaded = true;

		USaveGameSetup* saveGame = Cast<USaveGameSetup>(UGameplayStatics::CreateSaveGameObject(USaveGameSetup::StaticClass()));

		saveGame = Cast<USaveGameSetup>(UGameplayStatics::LoadGameFromSlot("GlobalGameStateData", state->GetFirstGamePlayer()->GetControllerId()));

		if (saveGame != nullptr)
		{
			FString playerId = "";
			FString playerName = "";
			ULocalPlayer* firstPlayer = state->GetFirstGamePlayer();

			if (firstPlayer != nullptr)
			{
				playerName = firstPlayer->GetNickname();

				FUniqueNetIdRepl uniqueNetId = firstPlayer->GetPreferredUniqueNetId();

				if (uniqueNetId.IsValid())
				{
					playerId = uniqueNetId->ToString();
				}
			}

			state->GamePlaySetup = saveGame->GamePlaySetup;
			state->GeneralOptions = saveGame->GeneralOptions;
			state->GeneralOptions.NumberOfPlayers = FMath::Min(state->GeneralOptions.NumberOfPlayers, GRIP_MAX_PLAYERS);
			state->AudioOptions = saveGame->AudioOptions;
			state->GraphicsOptions = saveGame->GraphicsOptions;
			state->InputControllerOptions = saveGame->InputControllerOptions;
		}

		state->VerifyInputOptions(false);
	}

#if !UE_BUILD_SHIPPING
	// Override some of the global game state from the play game mode if instructed to.

	APlayGameMode* gameMode = APlayGameMode::Get(instance);

	if (gameMode != nullptr &&
		gameMode->GameStateOverrides != nullptr)
	{
		if (gameMode->GameStateOverrides->OverrideGamePlaySetup == true)
		{
			state->GamePlaySetup = gameMode->GameStateOverrides->GamePlaySetup;
		}

		if (gameMode->GameStateOverrides->OverrideGeneralOptions == true)
		{
			state->GeneralOptions = gameMode->GameStateOverrides->GeneralOptions;
		}

		if (gameMode->GameStateOverrides->OverrideGraphicsOptions == true)
		{
			state->GraphicsOptions = gameMode->GameStateOverrides->GraphicsOptions;
		}

		if (gameMode->GameStateOverrides->OverrideAudioOptions == true)
		{
			state->AudioOptions = gameMode->GameStateOverrides->AudioOptions;
		}

		if (gameMode->GameStateOverrides->OverrideNavigationLayer == true)
		{
			state->TransientGameState.NavigationLayer = gameMode->GameStateOverrides->NavigationLayer;
		}
	}
#endif // !UE_BUILD_SHIPPING

#if UE_BUILD_DEBUG
	state->GraphicsOptions.ScreenMode = EScreenMode::FullScreenWindow;
#endif // UE_BUILD_DEBUG

}
