/**
*
* Driving surface characteristics.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* The vehicles need to understand something of the characteristics of the surfaces
* that they are driving on. Like friction, and how the tires interact with them
* both visually and how they sound too. These characteristics are held in a central
* data asset for the game, derived from UDrivingSurfaceCharacteristics. This asset
* is then referenced directly from each vehicle, so that it knows how to interact.
*
* There ought to be an instance of the ADrivingSurfaceProperties in each level too,
* which describes the average color of the level's dusty surfaces, and the average
* lighting levels too.
*
***********************************************************************************/

#include "effects/drivingsurfacecharacteristics.h"
#include "game/globalgamestate.h"

/**
* Get the tire friction for a surface type.
***********************************************************************************/

float UDrivingSurfaceCharacteristics::GetTireFriction(EGameSurface surfaceType) const
{
	return 0.9f;
}
