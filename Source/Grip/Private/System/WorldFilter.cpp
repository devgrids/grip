/**
*
* World filter component.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Used to strip actors from levels that don't belong to the current navigation layer.
*
***********************************************************************************/

#include "system/worldfilter.h"
#include "gamemodes/playgamemode.h"
#include "game/globalgamestate.h"
#include "vehicle/flippablevehicle.h"

/**
* Is the given actor valid for the given game state?
***********************************************************************************/

bool FWorldFilter::IsValid(AActor* actor, UGlobalGameState* gameState)
{
	if (GRIP_OBJECT_VALID(actor) == false ||
		actor->IsPendingKillPending() == true)
	{
		return false;
	}
	else
	{
		if (gameState != nullptr)
		{
			TArray<FName> avoid;
			FString acceptNavigation = gameState->TransientGameState.NavigationLayer;

			avoid.Emplace(TEXT("Weather"));

			for (FName& layer : actor->Layers)
			{
				for (FName& name : avoid)
				{
					if (layer == name)
					{
						if (gameState->IsLoaded() == true)
						{
							actor->Destroy();
							actor->ConditionalBeginDestroy();
						}

						return false;
					}
				}
			}

			for (FName& layer : actor->Layers)
			{
				FString layerString = layer.ToString();

				if (layerString == acceptNavigation &&
					acceptNavigation.Len() > 0)
				{
					return true;
				}
			}

			for (FName& layer : actor->Layers)
			{
				FString layerString = layer.ToString();

				if (layerString.EndsWith("Navigation") == true &&
					layerString != acceptNavigation &&
					acceptNavigation.Len() > 0)
				{
					if (gameState->IsLoaded() == true)
					{
						actor->Destroy();
						actor->ConditionalBeginDestroy();
					}

					return false;
				}
			}
		}

		return true;
	}
}

/**
* Is the given actor valid for the given navigation direction?
* Used in the Editor when no game state is present.
***********************************************************************************/

bool FWorldFilter::IsValid(AActor* actor, const FName& navigationLayer)
{
	if (navigationLayer == TEXT(""))
	{
		return true;
	}

	FString acceptNavigation = navigationLayer.ToString();

	for (FName& layer : actor->Layers)
	{
		FString layerString = layer.ToString();

		if (layerString.EndsWith("Navigation") == true &&
			layerString != acceptNavigation)
		{
			return false;
		}
	}

	return true;
}
