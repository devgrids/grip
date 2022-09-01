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

#pragma once

#include "system/gameconfiguration.h"
#include "Math/Vector.h"
#include "Attractable.generated.h"

/**
* Boilerplate class for the AttractableInterface.
***********************************************************************************/

UINTERFACE(MinimalAPI, meta = (CannotImplementInterfaceInBlueprint))
class UAttractableInterface : public UInterface
{
	GENERATED_UINTERFACE_BODY()
};

/**
* Interface class for the AttractableInterface.
***********************************************************************************/

class IAttractableInterface
{
	GENERATED_IINTERFACE_BODY()

public:

	// Is the attraction currently active?
	virtual bool IsAttractionActive() const = 0;

	// Get the attraction location.
	virtual FVector GetAttractionLocation() const = 0;

	// Get the attraction direction, or FVector::ZeroVector if no direction.
	virtual FVector GetAttractionDirection() const = 0;

	// Get the attraction distance range from the location.
	virtual float GetAttractionDistanceRange() const = 0;

	// Get the attraction minimum distance at which capture can start.
	virtual float GetAttractionMinCaptureDistanceRange() const = 0;

	// Get the attraction angle range from the direction.
	virtual float GetAttractionAngleRange() const = 0;

	// Is this attractor in range from a given location and direction?
	bool IsAttractorInRange(const FVector& fromLocation, const FVector& fromDirection, bool alreadyCaptured);

	// Is the attractor already attracting something?
	bool IsAttractorAttracting() const
	{ return Attracting; }

	// Set the actor that the attractor is attracting, or nullptr to stop attracting.
	void Attract(AActor* actor)
	{ AttractingActor = actor; Attracting = (actor != nullptr); }

private:

	// Is this attractor attracting another actor?
	bool Attracting = false;

	// The actor currently being attracted to this one.
	TWeakObjectPtr<AActor> AttractingActor;
};
