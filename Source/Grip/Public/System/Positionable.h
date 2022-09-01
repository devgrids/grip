/**
*
* Positionable interface.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* An interface to use for having objects clamped to the nearest surface in the scene.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "Positionable.generated.h"

/**
* Boilerplate class for the PositionableInterface.
***********************************************************************************/

UINTERFACE(MinimalAPI, meta = (CannotImplementInterfaceInBlueprint))
class UPositionableInterface : public UInterface
{
	GENERATED_UINTERFACE_BODY()
};

/**
* Interface class for the PositionableInterface.
***********************************************************************************/

class IPositionableInterface
{
	GENERATED_IINTERFACE_BODY()

public:

	// Determine the surface position / rotation for an actor.
	virtual bool DetermineSurfacePosition(FVector& position, FRotator& rotation, float radius, AActor* parent, ECollisionChannel channel = ECC_MAX, bool useRotation = false);
};
