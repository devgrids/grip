/**
*
* Advanced camera implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* An advanced camera component to give a lot of helper functionality to generalized
* camera work. It has 3 modes of operation, native, custom and mouse control, and
* has transition capabilities to interpolate between each mode when required. Native
* mode has the actor to which it's attached controlling its location and
* orientation. Custom, for when you want complete control like in cinematic cameras.
* And mouse, often used during the game's development but rarely in the field.
*
***********************************************************************************/

#include "camera/advancedcameracomponent.h"
#include "vehicle/flippablevehicle.h"
#include "gamemodes/menugamemode.h"

/**
* Construct an advanced camera component.
***********************************************************************************/

UAdvancedCameraComponent::UAdvancedCameraComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;

	bConstrainAspectRatio = false;
}
