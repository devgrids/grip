/**
*
* Vehicle impact effects.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* The vehicles need to understand something of the characteristics of the surfaces
* in the game for impact effects. These characteristics are held in a central
* data asset for the game, derived from UDrivingSurfaceImpactCharacteristics.
* This asset is then referenced directly from each vehicle, so that it knows how to
* produce such impact effects.
*
* The effects themselves, are generally spawned into the world via the
* AVehicleImpactEffect actor.
*
***********************************************************************************/

#include "effects/vehicleimpacteffect.h"
#include "vehicle/basevehicle.h"
