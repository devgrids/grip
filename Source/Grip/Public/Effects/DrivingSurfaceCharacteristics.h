/**
*
* Driving surface characteristics.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* The vehicles need to understand something of the characteristics of the surfaces
* that they are driving on. Like friction, and how the tires interact with them
* both visually and how they sound too. These characteristics are held in a central
* data asset for the game, derived from UDrivingSurfaceCharacteristics. This asset
* is then referenced directly from each vehicle, so that it knows how to interact.
*
* There ought to be an instance of the ADrivingSurfaceProperties in each level too,
* which describes the average color of the level's dusty surfaces, and the average
* lighting levels too.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "drivingsurfacecharacteristics.generated.h"

/**
* When you add new types, make sure you add to [Engine.PhysicsSettings] section
* of DefaultEngine.ini
***********************************************************************************/

UENUM(BlueprintType)
enum class EGameSurface : uint8
{
	Default = 0,
	Asphalt = 1,
	Dirt = 2,
	Water = 3,
	Wood = 4,
	Rock = 5,
	Metal = 6,
	Grass = 7,
	Gravel = 8,
	Sand = 9,
	Field = 10,
	Tractionless = 11,
	Vehicle = 12,
	Shield = 13,
	Snow = 14,
	Teleport = 15,
	Launched = 16 UMETA(Hidden),
	Num UMETA(Hidden)
};

/**
* The properties for driving surfaces for a particular level.
***********************************************************************************/

UCLASS(Blueprintable, ClassGroup = Vehicle)
class ADrivingSurfaceProperties : public AActor
{
	GENERATED_BODY()

public:

	// The ambient brightness of the level.
	UPROPERTY(EditAnywhere, Category = Aesthetics)
		float AmbientBrightness = 0.0f;

	// The average surface color of the level.
	UPROPERTY(EditAnywhere, Category = Aesthetics)
		FLinearColor SurfaceColor = FLinearColor::Black;

	// The average light level of the level.
	UPROPERTY(EditAnywhere, Category = Aesthetics)
		FLinearColor LightColor = FLinearColor::Black;
};

/**
* Structure for the properties of a driving surface with respect to how it
* interacts with the wheels of a vehicle.
***********************************************************************************/

USTRUCT()
struct FDrivingSurface
{
	GENERATED_USTRUCT_BODY()

public:

	// The surface type these properties apply to.
	UPROPERTY(EditAnywhere, Category = Surface)
		EGameSurface Surface;

	// Effect under the wheel.
	UPROPERTY(EditAnywhere, Category = Surface)
		UParticleSystem* Effect = nullptr;

	// Effect on the wheel.
	UPROPERTY(EditAnywhere, Category = Surface)
		UParticleSystem* FixedEffect = nullptr;

	// Effect under wheel for skidding.
	UPROPERTY(EditAnywhere, Category = Surface)
		UParticleSystem* WheelSkiddingEffect = nullptr;

	// Effect under wheel for wheel-spinning.
	UPROPERTY(EditAnywhere, Category = Surface)
		UParticleSystem* WheelSpinningEffect = nullptr;

	// Is the effect contactless? ie, it doesn't matter if the vehicle becomes airborne.
	UPROPERTY(EditAnywhere, Category = Surface)
		bool Contactless = false;

	// Is the effect cycled between vehicles? ie, it's rationalized and only a percentage of vehicles will show it at one time.
	UPROPERTY(EditAnywhere, Category = Surface)
		float CyclingRatio = 1.0f;

	// Sound cue for the surface traction sound.
	UPROPERTY(EditAnywhere, Category = Surface)
		USoundCue* TractionSound = nullptr;

	// Sound cue for the surface skidding sound.
	UPROPERTY(EditAnywhere, Category = Surface)
		USoundCue* SkiddingSound = nullptr;

	// Minimum speed to show FX on the surface.
	UPROPERTY(EditAnywhere, Category = Surface)
		float MinSpeed = 5.0f;

	// The tire friction coefficient on the surface.
	UPROPERTY(EditAnywhere, Category = Surface)
		float TireFriction = 1.0f;

	// Compare this surface with a surface type, used by TArray::FindByKey.
	bool operator == (const EGameSurface key) const
	{ return Surface == key; }
};

/**
* A dictionary driving surface properties so that we can determine how the wheels
* of a vehicle should behave when interacting with driving surfaces.
***********************************************************************************/

UCLASS(ClassGroup = Vehicle)
class UDrivingSurfaceCharacteristics : public UDataAsset
{
	GENERATED_BODY()

public:

	// The surface properties.
	UPROPERTY(EditAnywhere, Category = Characteristics)
		TArray<FDrivingSurface> Surfaces;

	// Get the tire friction for a surface type.
	float GetTireFriction(EGameSurface surfaceType) const;
};
