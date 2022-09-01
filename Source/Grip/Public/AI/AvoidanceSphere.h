/**
*
* Avoidance sphere implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* A sphere for describing a volume of space to be avoided.
*
***********************************************************************************/

#pragma once

#include "gameframework/actor.h"
#include "components/spherecomponent.h"
#include "system/avoidable.h"
#include "ai/advancedsplineactor.h"
#include "avoidancesphere.generated.h"

/**
* A sphere for describing a volume of space to be avoided.
***********************************************************************************/

UCLASS()
class GRIP_API AAvoidanceSphere : public AActor, public IAvoidableInterface
{
	GENERATED_BODY()

public:

	// Construct an avoidance sphere.
	AAvoidanceSphere();

	// Try to clear the obstacle very closely and not go too wide.
	UPROPERTY(EditAnywhere, Category = Avoidance)
		bool CloseClearance = false;

	// Always have any vehicles clear this obstacle towards the nearest path and never the other side.
	UPROPERTY(EditAnywhere, Category = Avoidance)
		bool ClearTowardsPath = false;

	// Force the AI system to reselect which spline the vehicle should follow when clearing this obstacle.
	UPROPERTY(EditAnywhere, Category = Avoidance)
		bool ReselectPursuitSplineWhenClear = false;

	// Is this obstacle just for antigravity vehicles?
	UPROPERTY(EditAnywhere, Category = Avoidance)
		EVehicleTypes VehicleTypes = EVehicleTypes::Both;

	// Sphere to avoid.
	UPROPERTY(EditAnywhere, Category = Avoidance, meta = (AllowPrivateAccess = "true"))
		USphereComponent* SphericalVolume = nullptr;

	// Is the attraction currently active?
	virtual bool IsAvoidanceActive() const override
	{ return true; }

	// Should vehicles brake to avoid this obstacle?
	virtual bool BrakeToAvoid() const override
	{ return true; }

	// Get the avoidance location.
	virtual FVector GetAvoidanceLocation() const override
	{ return (SphericalVolume->Mobility == EComponentMobility::Movable) ? GetActorLocation() : AvoidanceLocation; }

	// Get the avoidance normal.
	virtual FVector GetAvoidanceNormal() const override
	{ return AvoidanceNormal; }

	// Get the avoidance velocity in centimeters per second.
	virtual FVector GetAvoidanceVelocity() const override
	{ return FVector::ZeroVector; }

	// Get the avoidance radius from the location.
	virtual float GetAvoidanceRadius() const override
	{ return AvoidanceRadius; }

	// Get a direction vector that we prefer to clear the obstacle to, or ZeroVector if none.
	virtual FVector GetPreferredClearanceDirection() const override
	{ return (ClearTowardsPath == true) ? PathNormal : FVector::ZeroVector; }

	// Force the AI system to reselect which spline the vehicle should follow when clearing this obstacle?
	virtual bool GetReselectPursuitSplineWhenClear() const override
	{ return ReselectPursuitSplineWhenClear; }

	// Should the clearance of this obstacle be close?
	virtual bool EmployCloseClearance() const override
	{ return CloseClearance; }

	// Is this obstacle just for antigravity vehicles?
	virtual EVehicleTypes GetVehicleTypes() const override
	{ return VehicleTypes; }

private:

	// The direction to the nearest pursuit spline in world space.
	FVector PathNormal = FVector::ZeroVector;

	// The location to be avoided.
	FVector AvoidanceLocation = FVector::ZeroVector;

	// The direction in which this sphere should be passed.
	FVector AvoidanceNormal = FVector::ZeroVector;

	// The radius around the location to be avoided.
	float AvoidanceRadius = 0.0f;

	// The adjacent splines.
	UPROPERTY(Transient)
		TArray<TWeakObjectPtr<UAdvancedSplineComponent>> PursuitSplines;
};
