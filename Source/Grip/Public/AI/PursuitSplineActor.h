/**
*
* Pursuit spline actors.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Spline actors have functions for querying nearest splines for a given point in
* space. Generally, the is just one spline component attached to a spline actor.
* It also has support for enabling, always selecting and never selecting splines
* for bots at run-time, along with other, Editor-related functions.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "ai/pursuitsplinecomponent.h"
#include "ai/advancedsplineactor.h"
#include "pursuitsplineactor.generated.h"

/**
* Class for an pursuit spline actor, normally containing a single spline component.
***********************************************************************************/

UCLASS(ClassGroup = Navigation)
class GRIP_API APursuitSplineActor : public AAdvancedSplineActor
{
	GENERATED_BODY()

public:

	// Construct pursuit spline.
	APursuitSplineActor();

	// The point data specific to the pursuit spline.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Pursuit)
		TArray<FPursuitPointData> PointData;

	// The point extended data specific to the pursuit spline.
	UPROPERTY()
		TArray<FPursuitPointExtendedData> PointExtendedData;

	// Is this pursuit spline currently selected in the Editor?
	UPROPERTY(Transient, BlueprintReadOnly, Category = Pursuit)
		bool Selected;

	// Synchronize the pursuit point data with the points on the parent spline.
	UFUNCTION(BlueprintCallable, Category = Spline)
		bool SynchronisePointData();

	// Always select the path with the given name given the choice.
	UFUNCTION(BlueprintCallable, Category = Spline)
		static void AlwaysSelectPursuitPath(FString routeName, FString actorName, UObject* worldContextObject);

	// Never select the path with the given name given the choice.
	UFUNCTION(BlueprintCallable, Category = Spline)
		static void NeverSelectPursuitPath(FString routeName, FString actorName, UObject* worldContextObject);

	// Enable / disable the spline with the given name / route.
	UFUNCTION(BlueprintCallable, Category = Spline)
		static void EnablePursuitPath(FString routeName, FString actorName, bool enabled, UObject* worldContextObject);

	UFUNCTION(BlueprintImplementableEvent, BlueprintCallable, Category = "Default")
		void UpdateVisualisation();
};
