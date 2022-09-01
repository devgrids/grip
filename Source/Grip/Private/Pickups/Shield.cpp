/**
*
* Painkiller shield implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Shield pickup type, one of the pickups used by vehicles in the game.
*
***********************************************************************************/

#include "pickups/shield.h"
#include "pickups/homingmissile.h"
#include "vehicle/flippablevehicle.h"

/**
* Construct a shield.
***********************************************************************************/

AShield::AShield()
{
	PickupType = EPickupType::Shield;

	PrimaryActorTick.bCanEverTick = true;

	ActiveAudio = CreateDefaultSubobject<UAudioComponent>(TEXT("ActiveSound"));
}
