/**
*
* Wheel contact sensor implementation, use for wheels attached to vehicles.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Contact sensors provide information about nearest surface contacts for a wheel.
* They're paired for flippable vehicles so that we can detect contact but beneath
* and above any given wheel. They also provide suspension for standard vehicles
* and the hovering ability for antigravity vehicles.
*
***********************************************************************************/

#include "vehicle/vehiclecontactsensor.h"
#include "vehicle/basevehicle.h"
#include "gamemodes/basegamemode.h"
