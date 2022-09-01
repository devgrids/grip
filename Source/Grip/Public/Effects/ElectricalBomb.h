/**
*
* Electrical bomb implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* A bomb with electrical appearance, used in the main explosion for a destroyed
* vehicle to give the appearance of electrical tendrils during the explosion.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "pickups/pickupbase.h"
#include "effects/electricity.h"
#include "electricalbomb.generated.h"

UCLASS()
class GRIP_API AElectricalBomb : public AActor
{
	GENERATED_BODY()

public:

	// Construct an electrical bomb.
	AElectricalBomb();

	// The range of the electrical tendrils.
	UPROPERTY(EditAnywhere, Category = ElectricalBomb)
		float Range = 1000.0f * 100.0f;

	// The duration of the electrical bomb.
	UPROPERTY(EditAnywhere, Category = ElectricalBomb)
		float Duration;

	// Single, non-moving root.
	UPROPERTY(EditAnywhere, Category = ElectricalBomb)
		USceneComponent* StaticRoot = nullptr;

	// Single, moving root.
	UPROPERTY(EditAnywhere, Category = ElectricalBomb)
		USceneComponent* ElectricalRoot = nullptr;

	// The number of electrical tendrils in the bomb.
	UPROPERTY(EditAnywhere, Category = ElectricalBomb, meta = (UIMin = "1.0", UIMax = "100.0", ClampMin = "1.0", ClampMax = "100.0"))
		int32 NumStreaks = 10;

	// The properties of the electrical tendrils generated  by the bomb.
	UPROPERTY(EditAnywhere, Category = ElectricalBomb, meta = (AllowPrivateaccess = "true"))
		UElectricalStreakComponent* ElectricalStreak = nullptr;

	// The light for the center of the bomb.
	UPROPERTY(EditAnywhere, Category = ElectricalBomb, meta = (AllowPrivateaccess = "true"))
		UPointLightComponent* StartLocationLight = nullptr;

	// The light for where a tendril impacts a surface.
	UPROPERTY(EditAnywhere, Category = ElectricalBomb, meta = (AllowPrivateaccess = "true"))
		UPointLightComponent* EndLocationLight = nullptr;

	// The particle system to use for impacts.
	UPROPERTY(EditAnywhere, Category = ElectricalBomb)
		UParticleSystem* ElectricalImpactEffect = nullptr;

	// The current streaks representing the electrical tendrils of the bomb.
	UPROPERTY(Transient)
		TArray<UElectricalStreakComponent*> Streaks;

	// The current lights attached to the electrical tendrils of the bomb.
	UPROPERTY(Transient)
		TArray<UPointLightComponent*> Lights;
};
