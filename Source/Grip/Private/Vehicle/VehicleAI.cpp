/**
*
* Vehicle AI bot implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* The core of the AI bot implementation for vehicles. Most of the vehicle-specific
* AI code you'll find here in this module. Specifically though, the collision
* avoidance code you'll find in a separate VehicleAvoidance.cpp module.
*
***********************************************************************************/

#include "vehicle/flippablevehicle.h"
#include "ai/pursuitsplineactor.h"
#include "ai/avoidancesphere.h"
#include "game/globalgamestate.h"
#include "ai/playeraicontext.h"

/**
* Construct an AI context.
***********************************************************************************/

FVehicleAI::FVehicleAI()
{
	int32 rand = FMath::Rand();

	PursuitSplineWidthTime = FMath::FRand() * PI;
	PursuitSplineWidthOverTime = FMath::FRand() * 0.25f + 0.25f;
	WheelplayCycles = ((rand % 2) == 0) ? 3 + ((rand >> 3) % 3) : 0.0f;
	VariableSpeedOffset = FMath::FRand() * PI * 2.0f;

	for (float& time : DrivingModeTimes)
	{
		time = 0.0f;
	}
}

/**
* Lock the steering to spline direction?
***********************************************************************************/

void ABaseVehicle::SteeringToSplineDirection(bool locked, bool avoidStaticObjects)
{
}

/**
* Is an AI driver good for a launch?
***********************************************************************************/

bool ABaseVehicle::AIVehicleGoodForLaunch(float probability, float minimumSpeedKPH) const
{
	return false;
}
