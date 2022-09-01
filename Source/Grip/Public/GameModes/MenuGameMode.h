/**
*
* Menu game mode implementation.
*
* Original author: Nicky van de Groep, Jelle van der Gulik.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* The menu game mode to use for the game, specifically for the main user interface
* and is the C++ game mode used, with a blueprint wrapping it for actual use.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "gamemodes/basegamemode.h"
#include "menugamemode.generated.h"

/**
* The menu game mode to use for the game, specifically for the main user interface
* and is the C++ game mode used, with a blueprint wrapping it for actual use.
***********************************************************************************/

UCLASS(Abstract, Blueprintable)
class GRIP_API AMenuGameMode : public ABaseGameMode
{
	GENERATED_BODY()

public:

	// Get the menu game mode for the current world.
	static AMenuGameMode* Get(UObject* worldContextObject)
	{ return (worldContextObject == nullptr) ? nullptr : Cast<AMenuGameMode>(UGameplayStatics::GetGameMode(worldContextObject)); }

protected:

	// Do some post initialization just before the game is ready to play.
	virtual void PostInitializeComponents() override;

	// Do some initialization when the game is ready to play.
	virtual void BeginPlay() override;
};
