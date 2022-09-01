/**
*
* Advanced movement implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* An advanced movement component to give a lot of helper functionality to generalized
* movement work. Mostly, this will center on terrain avoidance.
*
***********************************************************************************/

#include "pickups/advancedmovementcomponent.h"
#include "gamemodes/basegamemode.h"

/**
* Construct an advanced movement component.
***********************************************************************************/

UAdvancedMovementComponent::UAdvancedMovementComponent()
{
	bUpdateOnlyIfRendered = false;
	bWantsInitializeComponent = true;

	Velocity = FVector::ZeroVector;
}
