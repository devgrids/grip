/**
*
* Vehicle impact effects.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* The vehicles need to understand something of the characteristics of the surfaces
* in the game for impact effects. These characteristics are held in a central
* data asset for the game, derived from UDrivingSurfaceImpactCharacteristics.
* This asset is then referenced directly from each vehicle, so that it knows how to
* produce such impact effects.
*
* The effects themselves, are generally spawned into the world via the
* AVehicleImpactEffect actor.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "drivingsurfacecharacteristics.h"
#include "vehicleimpacteffect.generated.h"

class ABaseVehicle;

/**
* Structure for the properties of a driving surface with respect to how it
* interacts when impacted by a vehicle.
***********************************************************************************/

USTRUCT()
struct FDrivingSurfaceImpact
{
	GENERATED_USTRUCT_BODY()

public:

	// The surface type these properties apply to.
	UPROPERTY(EditDefaultsOnly, Category = Surface)
		EGameSurface Surface;

	// Effect for a body impact.
	UPROPERTY(EditDefaultsOnly, Category = Surface)
		UParticleSystem* BodyEffect = nullptr;

	// Effect for a tire impact.
	UPROPERTY(EditDefaultsOnly, Category = Surface)
		UParticleSystem* TireEffect = nullptr;

	// Impact sound for body.
	UPROPERTY(EditDefaultsOnly, Category = Surface)
		USoundCue* BodySound = nullptr;

	// Impact sound for tire.
	UPROPERTY(EditDefaultsOnly, Category = Surface)
		USoundCue* TireSound = nullptr;

	// Compare this surface impact with a surface type, used by TArray::FindByKey.
	bool operator == (const EGameSurface key) const
	{ return Surface == key; }
};

/**
* A dictionary driving surface properties so that we can determine how the surface
* reacts to the impact of a vehicle.
***********************************************************************************/

UCLASS(ClassGroup = Vehicle)
class UDrivingSurfaceImpactCharacteristics : public UDataAsset
{
	GENERATED_BODY()

public:

	// The surface properties.
	UPROPERTY(EditAnywhere, Category = Characteristics)
		TArray<FDrivingSurfaceImpact> Surfaces;

	// Spawn an impact effect.
	static void SpawnImpact(ABaseVehicle* vehicle, const FDrivingSurfaceImpact& surface, bool tireImpact, const FVector& location, const FRotator& rotation, const FVector& velocity, const FVector& surfaceColor, const FVector& lightColor);
};

/**
* Effects component that represents a visual impact effect, but can be used in a
* general sense for any particle system component with a linear velocity.
***********************************************************************************/

UCLASS(ClassGroup = (Rendering, Common), meta = (BlueprintSpawnableComponent))
class UMovingParticleSystemComponent : public UParticleSystemComponent
{
	GENERATED_BODY()

public:

	// Indicate that this component requires ticking.
	UMovingParticleSystemComponent()
	{ PrimaryComponentTick.bCanEverTick = true; }
};
