/**
*
* Targetable interface.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* An interface to use for determining the target position on an object.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "Math/Vector.h"
#include "targetable.generated.h"

/**
* Boilerplate class for the TargetableInterface.
***********************************************************************************/

UINTERFACE(MinimalAPI, meta = (CannotImplementInterfaceInBlueprint))
class UTargetableInterface : public UInterface
{
	GENERATED_UINTERFACE_BODY()
};

/**
* Interface class for the TargetableInterface.
***********************************************************************************/

class ITargetableInterface
{
	GENERATED_IINTERFACE_BODY()

public:

	// Get the bulls-eye for targeting.
	virtual FVector GetTargetBullsEye() const = 0;
};
