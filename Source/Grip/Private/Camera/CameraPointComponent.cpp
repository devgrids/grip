/**
*
* Camera point components.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Camera points attached to vehicles normally, so that we can get a good view of
* the action when the cinematic camera is active.
*
***********************************************************************************/

#include "camera/camerapointcomponent.h"
#include "vehicle/flippablevehicle.h"
#include "gamemodes/basegamemode.h"

/**
* Construct a camera point component.
***********************************************************************************/

UCameraPointComponent::UCameraPointComponent()
{
	SetVisibleFlag(false);
	SetHiddenInGame(true);
}
