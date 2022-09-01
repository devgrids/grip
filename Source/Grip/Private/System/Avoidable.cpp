/**
*
* Avoidable interface.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* An interface to use for having objects be avoided by other objects, in the case
* of GRIP these objects are always avoided by AI bots. This includes other bots.
*
***********************************************************************************/

#include "system/avoidable.h"

/**
* Construct a UAvoidableInterface.
***********************************************************************************/

UAvoidableInterface::UAvoidableInterface(const FObjectInitializer& objectInitializer)
	: Super(objectInitializer)
{
}
