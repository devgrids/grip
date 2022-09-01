/**
*
* Cinematics director.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* All of the enums, structures and classes used for managing the cinematic camera,
* itself a director in coordinating the viewing of the action, encapsulated in the
* FCinematicsDirector class.
*
* The code that drives the cinematic camera sequencing at the end of a race and
* during the attract mode for the game.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "system/commontypes.h"
#include "camera/camerapointcomponent.h"

/**
* Class for managing the cinematic camera, a director in coordinating the viewing
* of the action.
***********************************************************************************/

class FCinematicsDirector
{
private:

	// A stock camera point to use on a vehicle that can be used for cinematic work. Reassigned between vehicles when required.
	UPROPERTY(Transient)
		UCameraPointComponent* StockCameraPoint = nullptr;

	// The current camera point, if any.
	UPROPERTY(Transient)
		UCameraPointComponent* CurrentCameraPoint = nullptr;
};
