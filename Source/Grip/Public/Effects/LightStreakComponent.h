/**
*
* Light streak implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Light streaks to replace the broken implementation of the ribbon emitter for
* particle systems. Used primarily on vehicles to accentuate speed, but also for
* sundry other things, like missile trails.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "system/mathhelpers.h"
#include "proceduralmeshcomponent.h"
#include "components/pointlightcomponent.h"
#include "lightstreakcomponent.generated.h"

/**
* A light streak component, specifically to replace the broken implementation of the
* ribbon emitter for particle systems
***********************************************************************************/

UCLASS(ClassGroup = Rendering, meta = (BlueprintSpawnableComponent))
class GRIP_API ULightStreakComponent : public USceneComponent
{
	GENERATED_BODY()

public:

	// Construct a light streak.
	ULightStreakComponent();

	// Use a streak?
	UPROPERTY(EditAnywhere, Category = Streak)
		bool Streak = true;

	// The width of the streak in centimeters.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		float Width = 25.0f;

	// The minimum distance between points before spawning a new point.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		float MinDistance = 100.0f;

	// The maximum distance between points before spawning a new point, or set to 0 if you want to spawn a new point every frame.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		float MaxDistance = 500.0f;

	// The maximum angle between points before spawning a new point.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		float MaxAngle = 3.0f;

	// The alpha scale.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak", UIMin = "0", UIMax = "1", ClampMin = "0", ClampMax = "1"))
		float Alpha = 1.0f;

	// Power to use to fade alpha over time (1 is a linear fade).
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		float AlphaFadePower = 2.0f;

	// The minimum speed at which the streak begins to show.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		float MinSpeed = 100.0f;

	// The maximum speed at which the streak has reached full opacity.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		float MaxSpeed = 500.0f;

	// How long it should take to fade in the alpha value.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		float FadeInTime = 0.0f;

	// The life time of the streak.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		float LifeTime = 0.5f;

	// How much to scale the tail of the streak by so that it can shrink (or grow) towards its end.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		float TailShrinkScale = 0.5f;

	// The amount of noise to apply to the streak.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		float StreakNoise = 0.0f;

	// Is the streak always facing towards the camera?
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		bool CameraFacing = true;

	// Even when camera facing, should the streak angle always be locked on the joint before it starts to face the camera later on?
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		bool LockJointAngle;

	// Should the streak fade off when the parent's direction deviates away from its velocity vector?
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		bool FadeStreakOnVelocityDeviation = false;

	// Should the streak fade off when the parent's direction deviates away from its velocity vector?
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		float FadeStreakOnVelocityDeviationAmount = 0.666f;

	// Should the streak be manually ticked (in order to improve the connection to a parent component in some cases).
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		bool ManualTick = false;

	// Should the streak be manually constructed (no automatic point adding).
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		bool ManualConstruction = false;

	// The color of the streak.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		FLinearColor StreakColour = FLinearColor(1.0f, 1.0f, 0.75f);

	// The color of the streak.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		FLinearColor StreakEndColour = FLinearColor(-1.0f, -1.0f, -1.0f);

	// The material to render the streak with (needs to be a specialist streak material).
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		UMaterialInterface* StreakMaterial = nullptr;

	// Use a flare?
	UPROPERTY(EditAnywhere, Category = Flare)
		bool Flare = true;

	// The size of the flare in centimeters.
	UPROPERTY(EditAnywhere, Category = Flare, meta = (EditCondition = "Flare"))
		float Size = 100.0f;

	// The aspect ratio of the flare.
	UPROPERTY(EditAnywhere, Category = Flare, meta = (EditCondition = "Flare"))
		float AspectRatio = 1.0f;

	// Should the flare auto-rotate as it moves across the screen?
	UPROPERTY(EditAnywhere, Category = Flare, meta = (EditCondition = "Flare"))
		bool AutoRotateFlare = true;

	// Should the flare rotate with the vehicle? (if not auto-rotating)
	UPROPERTY(EditAnywhere, Category = Flare, meta = (EditCondition = "Flare"))
		bool UseFlareRotation = true;

	// Should the flare fade off when the flare's direction deviates away from the camera's direction?
	UPROPERTY(EditAnywhere, Category = Flare, meta = (EditCondition = "Flare"))
		bool FadeFlareOnAngleDeviation = false;

	// Should the flare fade off when the flare's direction deviates away from the camera's direction?
	UPROPERTY(EditAnywhere, Category = Flare, meta = (EditCondition = "Flare"))
		float FadeFlareOnAngleDeviationAmount = 0.5f;

	// The material to render the flare with (needs to be a specialist flare material).
	UPROPERTY(EditAnywhere, Category = Flare, meta = (EditCondition = "Flare"))
		UMaterialInterface* FlareMaterial = nullptr;

	// The texture to render the flare with.
	UPROPERTY(EditAnywhere, Category = Flare, meta = (EditCondition = "Flare"))
		UTexture2D* FlareTexture = nullptr;

	// The color of the flare.
	UPROPERTY(EditAnywhere, Category = Flare, meta = (EditCondition = "Flare"))
		FLinearColor FlareColour = FLinearColor(1.0f, 1.0f, 0.75f);

	// The size of the flare in centimeters.
	UPROPERTY(EditAnywhere, Category = Flare, meta = (EditCondition = "Flare"))
		float CentralSize = 100.0f;

	// The aspect ratio of the flare.
	UPROPERTY(EditAnywhere, Category = Flare, meta = (EditCondition = "Flare"))
		float CentralAspectRatio = 1.0f;

	// The material to render the central flare with (needs to be a specialist flare material).
	UPROPERTY(EditAnywhere, Category = Flare, meta = (EditCondition = "Flare"))
		UMaterialInterface* CentralFlareMaterial = nullptr;

	// The texture to render the central flare with, if any.
	UPROPERTY(EditAnywhere, Category = Flare, meta = (EditCondition = "Flare"))
		UTexture2D* CentralFlareTexture = nullptr;

	// Set the controlling global amount for alpha and lifetime.
	void SetGlobalAmount(float alphaAmount, float lifeTimeAmount);

	// The particle system for the effect.
	UPROPERTY(Transient)
		UProceduralMeshComponent* Geometry = nullptr;

	// The dynamic material used for streaks.
	UPROPERTY(Transient)
		UMaterialInstanceDynamic* DynamicStreakMaterial = nullptr;

	// The dynamic material used for flares.
	UPROPERTY(Transient)
		UMaterialInstanceDynamic* DynamicFlareMaterial = nullptr;

	// The dynamic material used for central flares.
	UPROPERTY(Transient)
		UMaterialInstanceDynamic* DynamicCentralFlareMaterial = nullptr;

	// The standard material to be used for streaks.
	static UMaterialInterface* StandardStreakMaterial;

	// The standard material to be used for flares.
	static UMaterialInterface* StandardFlareMaterial;
};

/**
* A derivation of ULightStreakComponent with default properties that are pertinent
* to vehicles. Also optimized so that when they're not actually visible then they
* consume no CPU, which is most of the time.
***********************************************************************************/

UCLASS(ClassGroup = Rendering, meta = (BlueprintSpawnableComponent))
class GRIP_API UVehicleLightStreakComponent : public ULightStreakComponent
{
	GENERATED_BODY()

public:

	// Construct a vehicle light streak.
	UVehicleLightStreakComponent();
};
