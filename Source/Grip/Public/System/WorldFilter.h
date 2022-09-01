/**
*
* World filter stuff.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Used to strip actors from levels that don't belong to the current navigation layer.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"

class AActor;
class UGlobalGameState;

/**
* Class for filtering actors from levels.
***********************************************************************************/

class GRIP_API FWorldFilter
{
public:

	// Is the given actor valid for the given game state?
	static bool IsValid(AActor* actor, UGlobalGameState* gameState);

	// Is the given actor valid for the given navigation direction?
	// Used in the Editor when no game state is present.
	static bool IsValid(AActor* actor, const FName& navigationLayer);
};
