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

#include "effects/electricalbomb.h"
#include "gamemodes/basegamemode.h"

/**
* Construct an electrical bomb.
***********************************************************************************/

AElectricalBomb::AElectricalBomb()
{
	PrimaryActorTick.bCanEverTick = true;

	StaticRoot = CreateDefaultSubobject<USceneComponent>(TEXT("StaticRoot"));

	SetRootComponent(StaticRoot);

	ElectricalRoot = CreateDefaultSubobject<USceneComponent>(TEXT("ElectricalRoot"));
	GRIP_ATTACH(ElectricalRoot, StaticRoot, NAME_None);

	ElectricalStreak = CreateDefaultSubobject<UElectricalStreakComponent>(TEXT("ElectricalStreak"));
	GRIP_ATTACH(ElectricalStreak, ElectricalRoot, NAME_None);

	StartLocationLight = CreateDefaultSubobject<UPointLightComponent>(TEXT("StartLocationLight"));
	GRIP_ATTACH(StartLocationLight, ElectricalRoot, NAME_None);

	EndLocationLight = CreateDefaultSubobject<UPointLightComponent>(TEXT("EndLocationLight"));
	GRIP_ATTACH(EndLocationLight, StaticRoot, NAME_None);
}
