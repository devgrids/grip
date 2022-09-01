/**
*
* Pickup pad implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Pickup pads for vehicles to collect pickups from.
*
***********************************************************************************/

#include "pickups/pickup.h"
#include "vehicle/flippablevehicle.h"
#include "game/globalgamestate.h"
#include "ai/pursuitsplineactor.h"
#include "system/worldfilter.h"
#include "gamemodes/playgamemode.h"

/**
* Construct a pickup effect.
***********************************************************************************/

APickupEffect::APickupEffect()
{
	IdleEffect = CreateDefaultSubobject<UParticleSystemComponent>(TEXT("IdleEffect"));

	IdleEffect->bAutoDestroy = false;
	IdleEffect->bAutoActivate = false;
	IdleEffect->SetHiddenInGame(true);

	SetRootComponent(IdleEffect);

	PickedUpEffect = CreateDefaultSubobject<UParticleSystemComponent>(TEXT("PickedUpEffect"));

	PickedUpEffect->bAutoDestroy = false;
	PickedUpEffect->bAutoActivate = false;
	PickedUpEffect->SetHiddenInGame(true);
}

/**
* Construct a pickup.
***********************************************************************************/

APickup::APickup()
{
	CollisionSphere = CreateDefaultSubobject<USphereComponent>(TEXT("CollisionSphere"));

	SetRootComponent(CollisionSphere);

	CollisionSphere->SetCollisionEnabled(ECollisionEnabled::Type::NoCollision);
	CollisionSphere->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);

	PadMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("PadMesh"));
	GRIP_ATTACH(PadMesh, RootComponent, NAME_None);

	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickInterval = 0.1f;
	PrimaryActorTick.TickGroup = TG_DuringPhysics;
}
