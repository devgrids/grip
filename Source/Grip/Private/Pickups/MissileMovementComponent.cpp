/**
*
* Missile movement implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* We move the homing missile around using velocity changes and sub-stepping to
* ensure that we maintain a smooth movement arc. The UMissileMovementComponent
* does this work, inheriting from UAdvancedMovementComponent for some of that
* functionality.
*
***********************************************************************************/

#include "pickups/missilemovementcomponent.h"
#include "vehicle/flippablevehicle.h"
#include "gamemodes/basegamemode.h"

DEFINE_LOG_CATEGORY(GripLogMissile);

/**
* Construct a missile movement component.
***********************************************************************************/

UMissileMovementComponent::UMissileMovementComponent()
{
}
