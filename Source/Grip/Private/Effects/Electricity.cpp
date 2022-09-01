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

#include "effects/electricity.h"
#include "vehicle/flippablevehicle.h"
#include "uobject/constructorhelpers.h"
#include "gamemodes/basegamemode.h"

/**
* Some static data members.
***********************************************************************************/

UMaterialInterface* UElectricalStreakComponent::StandardStreakMaterial = nullptr;
UMaterialInterface* UElectricalStreakComponent::StandardFlareMaterial = nullptr;

/**
* Construct an electrical streak component.
***********************************************************************************/

UElectricalStreakComponent::UElectricalStreakComponent()
{
	bWantsInitializeComponent = true;
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_DuringPhysics;
}

/**
* Construct an electrical generator.
***********************************************************************************/

AElectricalGenerator::AElectricalGenerator()
{
	PrimaryActorTick.bCanEverTick = true;

	StartLocation = CreateDefaultSubobject<UBillboardComponent>(TEXT("StartLocation"));

	SetRootComponent(StartLocation);

#if WITH_EDITOR
	// Structure to hold one-time initialization
	struct FConstructorStatics
	{
		ConstructorHelpers::FObjectFinderOptional<UTexture2D> TargetIconSpawnObject;
		ConstructorHelpers::FObjectFinderOptional<UTexture2D> TargetIconObject;
		FName ID_TargetPoint;
		FText NAME_TargetPoint;
		FConstructorStatics()
			: TargetIconSpawnObject(TEXT("/Engine/EditorMaterials/TargetIconSpawn"))
			, TargetIconObject(TEXT("/Engine/EditorMaterials/TargetIcon"))
			, ID_TargetPoint(TEXT("TargetPoint"))
			, NAME_TargetPoint(NSLOCTEXT("SpriteCategory", "TargetPoint", "Target Points"))
		{ }
	};

	static FConstructorStatics ConstructorStatics;

	if (GRIP_OBJECT_VALID(StartLocation) == true)
	{
		StartLocation->Sprite = ConstructorStatics.TargetIconObject.Get();
		StartLocation->SpriteInfo.Category = ConstructorStatics.ID_TargetPoint;
		StartLocation->SpriteInfo.DisplayName = ConstructorStatics.NAME_TargetPoint;
		StartLocation->bIsScreenSizeScaled = true;

		StartLocation->SetRelativeScale3D(FVector(1.0f, 1.0f, 1.0f));
	}
#endif // WITH_EDITOR

	ElectricalStreak = CreateDefaultSubobject<UElectricalStreakComponent>(TEXT("ElectricalStreak"));

	GRIP_ATTACH(ElectricalStreak, StartLocation, NAME_None);

	StartLocationLight = CreateDefaultSubobject<UPointLightComponent>(TEXT("StartLocationLight"));

	GRIP_ATTACH(StartLocationLight, StartLocation, NAME_None);

	EndLocationLight = CreateDefaultSubobject<UPointLightComponent>(TEXT("EndLocationLight"));

	GRIP_ATTACH(EndLocationLight, StartLocation, NAME_None);
}

/**
* Enable electrical strikes.
***********************************************************************************/

void AElectricalGenerator::EnableStrikes() const
{
	TArray<UActorComponent*> components;

	GetComponents(UElectricalStreakComponent::StaticClass(), components);

	for (UActorComponent* component : components)
	{
		(Cast<UElectricalStreakComponent>(component))->StrikesEnabled = true;
		(Cast<UElectricalStreakComponent>(component))->AutoStrike = true;
	}
}

/**
* Disable electrical strikes.
***********************************************************************************/

void AElectricalGenerator::DisableStrikes() const
{
	TArray<UActorComponent*> components;

	GetComponents(UElectricalStreakComponent::StaticClass(), components);

	for (UActorComponent* component : components)
	{
		(Cast<UElectricalStreakComponent>(component))->StrikesEnabled = false;
		(Cast<UElectricalStreakComponent>(component))->AutoStrike = false;
	}
}
