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

#pragma once

#include "system/gameconfiguration.h"
#include "math/vector.h"
#include "system/commontypes.h"
#include "avoidable.generated.h"

/**
* Boilerplate class for the AvoidableInterface.
***********************************************************************************/

UINTERFACE(MinimalAPI, meta = (CannotImplementInterfaceInBlueprint))
class UAvoidableInterface : public UInterface
{
	GENERATED_UINTERFACE_BODY()
};

/**
* Interface class for the AvoidableInterface.
***********************************************************************************/

class IAvoidableInterface
{
	GENERATED_IINTERFACE_BODY()

public:

	// Is the attraction currently active?
	virtual bool IsAvoidanceActive() const = 0;

	// Should vehicles brake to avoid this obstacle?
	virtual bool BrakeToAvoid() const = 0;

	// Get the avoidance location.
	virtual FVector GetAvoidanceLocation() const = 0;

	// Get the avoidance normal.
	virtual FVector GetAvoidanceNormal() const
	{ return FVector::ZeroVector; }

	// Get the avoidance velocity in centimeters per second.
	virtual FVector GetAvoidanceVelocity() const = 0;

	// Get the avoidance radius from the location.
	virtual float GetAvoidanceRadius() const = 0;

	// Get a direction vector that we prefer to clear the obstacle to, or ZeroVector if none.
	virtual FVector GetPreferredClearanceDirection() const = 0;

	// Force the AI system to reselect which spline the vehicle should follow when clearing this obstacle?
	virtual bool GetReselectPursuitSplineWhenClear() const
	{ return false; }

	// Should the clearance of this obstacle be close?
	virtual bool EmployCloseClearance() const
	{ return false; }

	// Is this obstacle just for antigravity vehicles?
	virtual EVehicleTypes GetVehicleTypes() const
	{ return EVehicleTypes::Both; }
};
