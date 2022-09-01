/**
*
* Raptor Gatling gun implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Gatling gun pickup type, one of the pickups used by vehicles in the game.
*
***********************************************************************************/

#include "pickups/gatlinggun.h"
#include "vehicle/flippablevehicle.h"
#include "ui/hudwidget.h"
#include "gamemodes/basegamemode.h"

/**
* Construct a UGunHostInterface.
***********************************************************************************/

UGunHostInterface::UGunHostInterface(const FObjectInitializer& objectInitializer)
	: Super(objectInitializer)
{ }

/**
* Construct a gun.
***********************************************************************************/

AGatlingGun::AGatlingGun()
{
	PickupType = EPickupType::GatlingGun;

	PrimaryActorTick.bCanEverTick = true;

	BarrelSpinAudio = CreateDefaultSubobject<UAudioComponent>(TEXT("BarrelSpinAudio"));

	SetRootComponent(BarrelSpinAudio);
}
