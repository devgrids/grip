/**
*
* Pursuit spline components.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* This kind of spline is used primarily for AI bot track navigation, but also for
* cinematic camera work, weather determination and also for the Assassin missile
* navigation in the full version of the game. They're also critically important for
* race position determination.
*
***********************************************************************************/

#include "ai/pursuitsplinecomponent.h"
#include "ai/pursuitsplineactor.h"
#include "kismet/kismetmathlibrary.h"
#include "kismet/kismetmateriallibrary.h"
#include "system/mathhelpers.h"
#include "gamemodes/playgamemode.h"

DEFINE_LOG_CATEGORY(GripLogPursuitSplines);

/**
* Construct a pursuit spline component.
***********************************************************************************/

UPursuitSplineComponent::UPursuitSplineComponent()
{
}

/**
* Set the spline component for this spline mesh component.
***********************************************************************************/

void UPursuitSplineMeshComponent::SetupSplineComponent(UPursuitSplineComponent* splineComponent, int32 startPoint, int32 endPoint, bool selected)
{
}

/**
* Setup the rendering material for this spline mesh component.
***********************************************************************************/

void UPursuitSplineMeshComponent::SetupMaterial(bool selected)
{
}
