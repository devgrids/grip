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

#pragma once

#include "system/gameconfiguration.h"
#include "components/splinemeshcomponent.h"
#include "ai/advancedsplinecomponent.h"
#include "pursuitsplinecomponent.generated.h"

class UPursuitSplineComponent;
class APursuitSplineActor;
struct FPursuitSplineInstanceData;

DECLARE_LOG_CATEGORY_EXTERN(GripLogPursuitSplines, Log, All);

/**
* Pursuit spline types, Military currently unused.
***********************************************************************************/

UENUM(BlueprintType)
enum class EPursuitSplineType : uint8
{
	// General spline for vehicle use.
	General,

	// Spline for military use only, currently unused.
	Military,

	// Spline specifically designed to assist with missile guidance.
	MissileAssistance
};

/**
* A pursuit spline mesh component used solely for rendering pursuit splines. There
* is normally one mesh component for each segment of a pursuit spline component.
***********************************************************************************/

UCLASS(ClassGroup = Rendering, meta = (BlueprintSpawnableComponent))
class GRIP_API UPursuitSplineMeshComponent : public USplineMeshComponent
{
	GENERATED_BODY()

public:

	// Set the spline component for this spline mesh component.
	UFUNCTION(BlueprintCallable, Category = Mesh)
		void SetupSplineComponent(UPursuitSplineComponent* splineComponent, int32 startPoint, int32 endPoint, bool selected);

	// Setup the rendering material for this spline mesh component.
	UFUNCTION(BlueprintCallable, Category = Mesh)
		void SetupMaterial(bool selected);

private:

	// The spline component that we're rendering with this mesh.
	UPursuitSplineComponent* PursuitSplineComponent = nullptr;

	// The start control point index number.
	int32 StartPoint = 0;

	// The end control point index number.
	int32 EndPoint = 0;
};

/**
* Structure for user-specified point data for pursuit splines.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FPursuitPointData
{
	GENERATED_USTRUCT_BODY()

public:

	// The optimum speed in KPH (0 for full throttle) at this point for vehicles using this spline.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Spline)
		float OptimumSpeed = 0.0f;

	// The minimum speed in KPH (0 for none) at this point for vehicles using this spline.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Spline)
		float MinimumSpeed = 0.0f;

	// The maneuvering width in meters at this point for vehicles using this spline.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Spline)
		float ManeuveringWidth = 50.0f;

	// Is exterior weather allowed to be rendered at this point?
	// (we calculate undercover areas anyway so don't worry about those)
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Spline)
		bool WeatherAllowed = true;

	// Should projectiles follow the terrain at this point, or just follow the spline if not?
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Spline)
		bool ProjectilesFollowTerrain = true;
};

/**
* Structure for extended automatically-generated point data for pursuit splines.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FPursuitPointExtendedData
{
	GENERATED_USTRUCT_BODY()

public:

	// The distance along the spline at which the point is found.
	UPROPERTY()
		float Distance = 0.0f;

	// The distance around the master spline that this point matches.
	// Intentionally not necessarily the closest point if crossovers and loops are present.
	UPROPERTY()
		float MasterSplineDistance = -1.0f;

	// The maximum diameter of the tunnel if inside a tunnel.
	UPROPERTY()
		float MaxTunnelDiameter = 0.0f;

	// The raw, unfiltered exterior weather allowed to be rendered at this point? (< 1 means not)
	UPROPERTY()
		float RawWeatherAllowed = 0.0f;

	// The filtered, more natural exterior weather allowed to be rendered at this point? (< 1 means not)
	UPROPERTY()
		float UseWeatherAllowed = 0.0f;

	// The index to identify the curvature of the spline in environment space.
	// (i.e. which environment index would you naturally drive along).
	UPROPERTY()
		int32 CurvatureIndex = 0;

	// The raw, unfiltered ground index into the environment distances.
	UPROPERTY()
		int32 RawGroundIndex = 0;

	// The filtered, more natural ground index into the environment distances.
	UPROPERTY()
		int32 UseGroundIndex = 0;

	// Where is the ground relative to this point, in world space?
	// NB. Ground is the closest point, not necessarily below.
	UPROPERTY()
		FVector RawGroundOffset = FVector::ZeroVector;

	// Where is the ground relative to this point, in world space?
	// NB. Ground is the closest point, not necessarily below.
	UPROPERTY()
		FVector UseGroundOffset = FVector::ZeroVector;

	// How far away are the nearest objects to this point for a number of samples, in centimeters.
	UPROPERTY()
		TArray<float> EnvironmentDistances;

	// Does the left-hand driving surface have open edge? (therefore, don't drive over it)
	UPROPERTY()
		bool OpenLeft = false;

	// Does the left-hand driving surface have open edge? (therefore, don't drive over it)
	UPROPERTY()
		bool OpenRight = false;

	// The orientation, cached here for speed.
	UPROPERTY()
		FQuat Quaternion = FQuat::Identity;

	// Does this point reside over level ground?
	bool IsLevelGround()
	{ return (EnvironmentDistances[UseGroundIndex] < 25.0f * 100.0f && UseGroundIndex >= (NumDistances >> 1) - (NumDistances >> 4) && UseGroundIndex <= (NumDistances >> 1) + (NumDistances >> 4)); }

	// Does this point reside under level ceiling?
	bool IsLevelCeiling()
	{ return (EnvironmentDistances[UseGroundIndex] < 25.0f * 100.0f && (UseGroundIndex >= (NumDistances - (NumDistances >> 4)) || UseGroundIndex <= (NumDistances >> 4))); }

	// Get the angle difference between to environment samples.
	static float DifferenceInDegrees(int32 indexFrom, int32 indexTo);

	// The number of environment distances what we sample and store.
	static const int32 NumDistances = 32;
};

/**
* Class for a pursuit spline component, normally one per actor.
***********************************************************************************/

UCLASS(ClassGroup = Navigation, meta = (BlueprintSpawnableComponent))
class GRIP_API UPursuitSplineComponent : public UAdvancedSplineComponent
{
	GENERATED_BODY()

public:

	// Construct a pursuit spline component.
	UPursuitSplineComponent();

	// Always select this spline if enabled?
	UPROPERTY(EditAnywhere, Category = Spline)
		bool AlwaysSelect = false;

	// Can the spline be used for guiding missiles?
	UPROPERTY(EditAnywhere, Category = Spline)
		bool SuitableForMissileGuidance = true;

	// Does this spline contain a bundle of pickups?
	UPROPERTY(EditAnywhere, Category = Spline)
		bool ContainsPickups = false;

	// Is this spline a shortcut?
	UPROPERTY(EditAnywhere, Category = Spline)
		bool IsShortcut = false;

	// Is this spline for careful driving?
	UPROPERTY(EditAnywhere, Category = Spline)
		bool CarefulDriving = false;

	// The type of spline.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = Spline)
		EPursuitSplineType Type = EPursuitSplineType::General;

	// What probability is there of this branch being selected (ostensibly between 0 and 1)?
	UPROPERTY(EditAnywhere, Category = Spline)
		float BranchProbability = 1.0f;

	// Get the maneuvering width at a point along a spline.
	UFUNCTION(BlueprintCallable, Category = Spline)
		float GetWidthAtSplinePoint(int32 point) const
	{ return 0.0f; }

	// Get the optimum speed at a point along a spline.
	UFUNCTION(BlueprintCallable, Category = Spline)
		float GetOptimumSpeedAtSplinePoint(int32 point) const
	{ return 0.0f; }

	// Get the minimum speed at a point along a spline.
	UFUNCTION(BlueprintCallable, Category = Spline)
		float GetMinimumSpeedAtSplinePoint(int32 point) const
	{ return 0.0f; }

	UFUNCTION(BlueprintCallable, Category = Spline)
		void EmptySplineMeshes()
	{ }
};

/**
* Structure used to store spline data during RerunConstructionScripts.
***********************************************************************************/

USTRUCT()
struct FPursuitSplineInstanceData : public FSplineInstanceData
{
	GENERATED_BODY()

public:

	FPursuitSplineInstanceData() = default;

	explicit FPursuitSplineInstanceData(const UPursuitSplineComponent* SourceComponent)
		: FSplineInstanceData(SourceComponent)
	{ }

	virtual ~FPursuitSplineInstanceData() = default;

	virtual void ApplyToComponent(UActorComponent* Component, const ECacheApplyPhase CacheApplyPhase) override
	{
		FSplineInstanceData::ApplyToComponent(Component, CacheApplyPhase);
		CastChecked<UPursuitSplineComponent>(Component)->ApplyComponentInstanceData(this, (CacheApplyPhase == ECacheApplyPhase::PostUserConstructionScript));
	}

	UPROPERTY()
		bool bClosedLoop = false;

	UPROPERTY()
		bool bClosedLoopPreUCS = false;

	UPROPERTY()
		EPursuitSplineType Type = EPursuitSplineType::General;

	UPROPERTY()
		EPursuitSplineType TypePreUCS = EPursuitSplineType::General;
};
