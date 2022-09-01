/**
*
* Scorpion homing missile implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Homing missile pickup type, one of the pickups used by vehicles in the game.
*
***********************************************************************************/

#include "pickups/homingmissile.h"
#include "vehicle/flippablevehicle.h"
#include "gamemodes/basegamemode.h"

/**
* Construct a UMissileHostInterface.
***********************************************************************************/

UMissileHostInterface::UMissileHostInterface(const FObjectInitializer& objectInitializer)
	: Super(objectInitializer)
{ }

/**
* Construct a homing missile.
***********************************************************************************/

AHomingMissile::AHomingMissile()
{
	PickupType = EPickupType::HomingMissile;

	MissileMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("MissileMesh"));

	SetRootComponent(MissileMesh);

	MissileMesh->bReturnMaterialOnMove = true;

	MissileMovement = CreateDefaultSubobject<UMissileMovementComponent>(TEXT("MissileMovement"));

	RocketTrail = CreateDefaultSubobject<UGripTrailParticleSystemComponent>(TEXT("RocketTrail"));
	GRIP_ATTACH(RocketTrail, RootComponent, NAME_None);

	RocketTrail->bAutoDestroy = false;
	RocketTrail->bAutoActivate = false;
	RocketTrail->SetHiddenInGame(true);

	RocketLightStreak = CreateDefaultSubobject<ULightStreakComponent>(TEXT("RocketLightStreak"));
	GRIP_ATTACH(RocketLightStreak, RootComponent, NAME_None);

	RocketLight = CreateDefaultSubobject<UPointLightComponent>(TEXT("RocketLight"));
	GRIP_ATTACH(RocketLight, RootComponent, NAME_None);

	RocketLight->bAutoActivate = false;
	RocketLight->SetHiddenInGame(true);

	ExplosionForce = CreateDefaultSubobject<URadialForceComponent>(TEXT("ExplosionForce"));
	ExplosionForce->bAutoActivate = false;
	GRIP_ATTACH(ExplosionForce, RootComponent, NAME_None);

	PrimaryActorTick.bCanEverTick = true;
}
