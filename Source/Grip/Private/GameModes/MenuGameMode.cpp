/**
*
* Menu game mode implementation.
*
* Original author: Nicky van de Groep, Jelle van der Gulik.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
***********************************************************************************/

#include "gamemodes/menugamemode.h"
#include "game/globalgamestate.h"
#include "vehicle/flippablevehicle.h"
#include "blueprint/widgetblueprintlibrary.h"
#include "audiodevice.h"

/**
* Do some initialization when the menu is ready to run.
***********************************************************************************/

void AMenuGameMode::PostInitializeComponents()
{
	Super::PostInitializeComponents();

	if (GetWorld() != nullptr &&
		GetWorld()->GetGameViewport() != nullptr)
	{
		GetWorld()->GetGameViewport()->SetForceDisableSplitscreen(true);
	}
}

/**
* Do some initialization when the game is ready to play.
***********************************************************************************/

void AMenuGameMode::BeginPlay()
{
	Super::BeginPlay();

#if GRIP_ENGINE_EXTENDED_MODIFICATIONS
	if (FAudioDeviceHandle audioDevice = GetWorld()->GetAudioDevice())
	{
		audioDevice->SetMirroredAudio(false);
	}
#endif // GRIP_ENGINE_EXTENDED_MODIFICATIONS

	APlayerController* controller = GetWorld()->GetFirstPlayerController();

	if (controller != nullptr)
	{
		controller->bEnableClickEvents = true;
		controller->bEnableMouseOverEvents = true;
	}

	for (int32 i = 0; i < GRIP_MAX_LOCAL_PLAYERS; i++)
	{
		controller = UGameplayStatics::GetPlayerController(this, i);

		if (controller != nullptr)
		{
			UWidgetBlueprintLibrary::SetInputMode_GameAndUIEx(controller, nullptr, EMouseLockMode::LockOnCapture);

			controller->bShowMouseCursor = true;
		}
	}
}
