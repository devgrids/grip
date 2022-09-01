/**
*
* Attractable interface.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* An interface to use for having objects be attractive to other objects, in the case
* of GRIP these objects are always attractive to AI bots.
*
* For example you might have speed pads be attractive to AI bots by simply having
* the speed pads inherit the IAttractableInterface class and define its virtual
* functions.
*
***********************************************************************************/

#include "system/attractable.h"
#include "system/mathhelpers.h"

/**
* Construct a UAttractableInterface, has to be defined here as can't be defined
* in the header file for some reason.
***********************************************************************************/

UAttractableInterface::UAttractableInterface(const FObjectInitializer& objectInitializer)
	: Super(objectInitializer)
{ }

/**
* Is this attractor in range from a given location and direction?
***********************************************************************************/

bool IAttractableInterface::IsAttractorInRange(const FVector& fromLocation, const FVector& fromDirection, bool alreadyCaptured)
{
	return false;
}
