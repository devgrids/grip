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

#include "effects/lightstreakcomponent.h"
#include "vehicle/flippablevehicle.h"
#include "uobject/constructorhelpers.h"

UMaterialInterface* ULightStreakComponent::StandardStreakMaterial = nullptr;
UMaterialInterface* ULightStreakComponent::StandardFlareMaterial = nullptr;

/**
* Construct a light streak component.
***********************************************************************************/

ULightStreakComponent::ULightStreakComponent()
{
	bWantsInitializeComponent = true;
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_DuringPhysics;
}

/**
* Construct a vehicle light streak component.
***********************************************************************************/

UVehicleLightStreakComponent::UVehicleLightStreakComponent()
{
	Width = 15.0f;
	MinDistance = 25.0f;
	MaxDistance = 100.0f;
	MaxAngle = 0.5f;
	MinSpeed = 300.0f;
	MaxSpeed = 550.0f;
	LifeTime = 0.15f;
	TailShrinkScale = 0.25f;
	StreakNoise = 0.666f;
	FadeStreakOnVelocityDeviation = true;
	FadeStreakOnVelocityDeviationAmount = 0.95f;
	StreakColour = FLinearColor(1.0f, 0.195f, 0.0f, 1.0f);
	Size = 150.0f;
	FadeFlareOnAngleDeviation = true;
	FadeFlareOnAngleDeviationAmount = 0.666f;
	FlareColour = FLinearColor(1.0f, 0.195f, 0.0f, 1.0f);

	SetRelativeRotation(FRotator(0.0f, 180.0f, 0.0f));

	PrimaryComponentTick.bStartWithTickEnabled = false;
	PrimaryComponentTick.SetTickFunctionEnable(false);
}

/**
* Set the controlling global amount for alpha and lifetime.
***********************************************************************************/

void ULightStreakComponent::SetGlobalAmount(float alphaAmount, float lifeTimeAmount)
{
}
