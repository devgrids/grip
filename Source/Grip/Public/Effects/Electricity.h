/**
*
* Electricity implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* All of the structures and classes used to render electricity within the game.
* We have single electrical streak components, and also electrical generators to
* generate multiple streaks either on a continuous basis or just for short periods.
* This is used for effects with some of the levels, and also for the vehicle
* destroyed explosion.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "effects/lightstreakcomponent.h"
#include "electricity.generated.h"

/**
* Structure for a minimum and maximum range.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FMinMax
{
	GENERATED_USTRUCT_BODY()

public:

	FMinMax() = default;

	FMinMax(float value)
		: Minimum(value)
		, Maximum(value)
		, Value(value)
	{ }

	FMinMax(float minimum, float maximum)
		: Minimum(minimum)
		, Maximum(maximum)
		, Value((minimum + maximum) * 0.5f)
	{ }

	// Get the current value within the minimum and maximum range.
	float Get() const
	{ return Value; }

	// Get a random number within the minimum and maximum range.
	float GetRandom() const
	{ return (Minimum == Maximum) ? Minimum : FMath::FRandRange(Minimum, Maximum); }

	// Generate a random number within the minimum and maximum range and store it away in our internal value.
	float GenerateRandom()
	{ Value = GetRandom(); return Value; }

	// The minimum extent of the range.
	UPROPERTY(EditAnywhere, Category = Default)
		float Minimum = 0.0f;

	// The maximum extent of the range.
	UPROPERTY(EditAnywhere, Category = Default)
		float Maximum = 1.0f;

	// The current internal value within the minimum and maximum range.
	float Value = 0.0f;
};

/**
*
***********************************************************************************/

UCLASS(ClassGroup = Rendering, meta = (BlueprintSpawnableComponent))
class GRIP_API UElectricalStreakComponent : public USceneComponent
{
	GENERATED_BODY()

public:

	UElectricalStreakComponent();

	// Automatically restrike after the Post Delay.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		bool AutoStrike = true;

	// The width of the streak in centimeters.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		FMinMax Width = FMinMax(25.0f);

	// The deviation away from the end location.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		FMinMax Deviation = FMinMax(0.15f, 0.25f);

	// The maximum number of points used to render each electrical tendril.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		int32 NumPoints = 256;

	// The number of meters between each point.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		float NumMetresPerPoint = 1.0f;

	// The alpha scale.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak", UIMin = "0", UIMax = "1", ClampMin = "0", ClampMax = "1"))
		float Alpha = 1.0f;

	// Power to use to fade alpha over time (1 is a linear fade).
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		float AlphaFadePower = 0.5f;

	// The life time of the streak.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		FMinMax LifeTime = FMinMax(0.15f, 0.25f);

	// The initial delay before striking electricity.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		FMinMax InitialDelay = FMinMax(0.0f);

	// The delay after striking electricity.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		FMinMax PostDelay = FMinMax(0.0f);

	// The amount to scale the electrical tendrils by over its life time.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		FMinMax LifeShrinkScale = FMinMax(0.25f, 0.5f);

	// The alpha scale for the electrical tendrils.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak", UIMin = "0", UIMax = "1", ClampMin = "0", ClampMax = "1"))
		float TendrilAlphaScale = 1.0f;

	// The shrink scale for the electrical tendrils.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak", UIMin = "0", UIMax = "1", ClampMin = "0", ClampMax = "1"))
		float TendrilShrinkScale = 0.9f;

	// The power to apply to time values used in rendering the tendrils.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak", UIMin = "0", UIMax = "10", ClampMin = "0", ClampMax = "10"))
		float TendrilTimePower = 0.25f;

	// The power to apply to the length ratios used in rendering the tendrils.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak", UIMin = "0", UIMax = "10", ClampMin = "0", ClampMax = "10"))
		float TendrilLengthPower = 1.5f;

	// The color of the streak.
	UPROPERTY(EditAnywhere, Category = Streak, meta = (EditCondition = "Streak"))
		FLinearColor StreakColour = FLinearColor(15.0f, 15.0f, 50.0f);

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
		bool AutoRotateFlare = false;

	// The depth fade of the flare.
	UPROPERTY(EditAnywhere, Category = Flare, meta = (EditCondition = "Flare"))
		float FlareDepthFade = 100.0f;

	// The material to render the streak with (needs to be a specialist streak material).
	UPROPERTY(EditAnywhere, Category = Flare, meta = (EditCondition = "Flare"))
		UMaterialInterface* FlareMaterial = nullptr;

	// The texture to render the streak with.
	UPROPERTY(EditAnywhere, Category = Flare, meta = (EditCondition = "Flare"))
		UTexture2D* FlareTexture = nullptr;

	// The color of the flare.
	UPROPERTY(EditAnywhere, Category = Flare, meta = (EditCondition = "Flare"))
		FLinearColor FlareColour = FLinearColor(16.0f, 16.0f, 40.0f);

	// Are strikes currently enabled?
	bool StrikesEnabled = true;

	// The particle system for the effect.
	UPROPERTY(Transient)
		UProceduralMeshComponent* Geometry = nullptr;

	// The dynamic material used for streaks.
	UPROPERTY(Transient)
		UMaterialInstanceDynamic* DynamicStreakMaterial = nullptr;

	// The dynamic material used for flares.
	UPROPERTY(Transient)
		UMaterialInstanceDynamic* DynamicFlareMaterial = nullptr;

	// The standard material to be used for streaks.
	static UMaterialInterface* StandardStreakMaterial;

	// The standard material to be used for flares.
	static UMaterialInterface* StandardFlareMaterial;
};

/**
* An end location component to use with electricity generation.
***********************************************************************************/

UCLASS(ClassGroup = Rendering, meta = (BlueprintSpawnableComponent))
class GRIP_API UEndLocationComponent : public USceneComponent
{
	GENERATED_BODY()

public:

	// The probability of being selected.
	UPROPERTY(EditAnywhere, Category = Default)
		float Probability = 1.0f;
};

/**
* An electrical generator, used to generate electrical streak components.
***********************************************************************************/

UCLASS()
class GRIP_API AElectricalGenerator : public AActor
{
	GENERATED_BODY()

public:

	// Construct an electrical generator.
	AElectricalGenerator();

	// The number of electrical tendrils.
	UPROPERTY(EditAnywhere, Category = Generator)
		int32 NumStreaks = 3;

	// The start location for electrical generation.
	UPROPERTY(VisibleAnywhere, Category = Generator, meta = (AllowPrivateaccess = "true"))
		UBillboardComponent* StartLocation = nullptr;

	// The base properties of the electrical streaks that will be generated.
	UPROPERTY(VisibleAnywhere, Category = Generator, meta = (AllowPrivateaccess = "true"))
		UElectricalStreakComponent* ElectricalStreak = nullptr;

	// The base properties of the start location light.
	UPROPERTY(VisibleAnywhere, Category = Generator, meta = (AllowPrivateaccess = "true"))
		UPointLightComponent* StartLocationLight = nullptr;

	// The base properties of the end location lights that will be generated.
	UPROPERTY(VisibleAnywhere, Category = Generator, meta = (AllowPrivateaccess = "true"))
		UPointLightComponent* EndLocationLight = nullptr;

	// Enable electrical strikes.
	UFUNCTION(BlueprintCallable, Category = Generator)
		void EnableStrikes() const;

	// Disable electrical strikes.
	UFUNCTION(BlueprintCallable, Category = Generator)
		void DisableStrikes() const;

	// Strike with electricity.
	UFUNCTION(BlueprintImplementableEvent, Category = Generator)
		void Strike(const FVector& location, const FVector& surfaceNormal, const FVector& strikeNormal, const FVector& mergedNormal, const FVector& reflectionNormal);

	// The streaks to use to render the electricity.
	UPROPERTY(Transient)
		TArray<UElectricalStreakComponent*> AdditionalStreaks;

	// The point lights used in rendering the electricity.
	UPROPERTY(Transient)
		TArray<UPointLightComponent*> AdditionalPointLights;
};
