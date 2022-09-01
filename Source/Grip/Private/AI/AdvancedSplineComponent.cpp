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

#include "ai/advancedsplinecomponent.h"
#include "system/mathhelpers.h"

/**
* Construct an advanced spline component.
***********************************************************************************/

UAdvancedSplineComponent::UAdvancedSplineComponent()
{
	UPrimitiveComponent::SetCollisionEnabled(ECollisionEnabled::Type::NoCollision);
	UPrimitiveComponent::SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);

	SetGenerateOverlapEvents(false);

	Mobility = EComponentMobility::Movable;

	// Grab the actor's name and store it locally for easier diagnostic work.

	AActor* actor = GetOwner();

	if (actor != nullptr)
	{
		ActorName = actor->GetName();
	}
}
