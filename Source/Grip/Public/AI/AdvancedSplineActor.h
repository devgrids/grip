/**
*
* Advanced spline actors.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Spline actors have functions for querying nearest splines for a given point in
* space. Generally, the is just one spline component attached to a spline actor.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "ai/advancedsplinecomponent.h"
#include "advancedsplineactor.generated.h"

/**
* Class for an advanced spline actor, normally containing a single spline component.
***********************************************************************************/

UCLASS(ClassGroup = Navigation)
class GRIP_API AAdvancedSplineActor : public AActor
{
	GENERATED_BODY()

public:

	// Is the spline visualization currently enabled?
	UPROPERTY(EditAnywhere, Category = Rendering)
		bool VisualisationEnabled = true;
};
