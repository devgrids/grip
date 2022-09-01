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

#include "ai/pursuitsplineactor.h"
#include "system/worldfilter.h"
#include "game/globalgamestate.h"
#include "gamemodes/playgamemode.h"

/**
* Construct pursuit spline.
***********************************************************************************/

APursuitSplineActor::APursuitSplineActor()
{
}

/**
* Always select the spline with the given name / route given the choice.
***********************************************************************************/

void APursuitSplineActor::AlwaysSelectPursuitPath(FString routeName, FString actorName, UObject* worldContextObject)
{
}

/**
* Never select the spline with the given name / route given the choice.
***********************************************************************************/

void APursuitSplineActor::NeverSelectPursuitPath(FString routeName, FString actorName, UObject* worldContextObject)
{
}

/**
* Enable / disable the spline with the given name / route.
***********************************************************************************/

void APursuitSplineActor::EnablePursuitPath(FString routeName, FString actorName, bool enabled, UObject* worldContextObject)
{
}

/**
* Synchronize the pursuit point data with the points on the parent spline.
***********************************************************************************/

bool APursuitSplineActor::SynchronisePointData()
{
	return false;
}
