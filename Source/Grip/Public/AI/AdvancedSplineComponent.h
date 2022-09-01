/**
*
* Advanced spline components.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Spline components with extended functionality over USplineComponent but not really
* much in the way of new properties. It performs some extended geometry analysis
* on splines, including GetNearestDistance which returns the nearest position on a
* spline for a given position in space.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "components/splinecomponent.h"
#include "advancedsplinecomponent.generated.h"

/**
* Class for an advanced spline component, normally one per actor.
***********************************************************************************/

UCLASS(ClassGroup = Navigation, meta = (BlueprintSpawnableComponent))
class GRIP_API UAdvancedSplineComponent : public USplineComponent
{
	GENERATED_BODY()

public:

	// Construct an advanced spline component.
	UAdvancedSplineComponent();

	// What is the name of the route this spline it linked to?
	UPROPERTY(EditAnywhere, Category = AdvancedSpline)
		FString RouteName;

	// Is the spline currently enabled?
	UPROPERTY(EditAnywhere, Category = AdvancedSpline)
		bool Enabled = true;

	// Get the index after the given point clamped to the number of points on the spline.
	UFUNCTION(BlueprintCallable, Category = AdvancedSpline)
		int32 ClampedNextIndex(int32 index) const
	{ return (index + 1) % GetNumberOfSplinePoints(); }

	// Draw a box for debugging purposes.
	UFUNCTION(BlueprintCallable, Category = AdvancedSpline)
		void DrawBox(FBox const& Box, FColor const& Color)
	{ DrawDebugSolidBox(GetWorld(), Box, Color); }

	// The parent actor's name, for debugging.
	FString ActorName;
};
