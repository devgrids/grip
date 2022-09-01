/**
*
* Speed pad implementation.
*
* Original author: Nicky van de Groep.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Speed pads push the vehicles forwards at higher speed when they traverse over
* them. They inherit from the IAttractableInterface interface and so the AI bot
* code will automatically detect them and head towards them where appropriate.
*
***********************************************************************************/

#include "pickups/speedpad.h"
#include "vehicle/flippablevehicle.h"
#include "gamemodes/playgamemode.h"

/**
* Sets default values for this actor's properties.
***********************************************************************************/

ASpeedPad::ASpeedPad()
{
	CollisionBox = CreateDefaultSubobject<UBoxComponent>(TEXT("CollisionBox"));

	SetRootComponent(CollisionBox);

	CollisionBox->SetCollisionEnabled(ECollisionEnabled::Type::NoCollision);
	CollisionBox->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);

	PadMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("PadMesh"));
	GRIP_ATTACH(PadMesh, RootComponent, NAME_None);

	CollectedAudio = CreateDefaultSubobject<UAudioComponent>(TEXT("CollectedSound"));
	GRIP_ATTACH(CollectedAudio, RootComponent, NAME_None);

	CollectedEffect = CreateDefaultSubobject<UParticleSystemComponent>(TEXT("CollectedEffect"));

	CollectedEffect->bAutoDestroy = false;
	CollectedEffect->bAutoActivate = false;
	CollectedEffect->SetHiddenInGame(true);

	GRIP_ATTACH(CollectedEffect, RootComponent, NAME_None);

	CollectedEffect->SetWorldScale3D(FVector::OneVector);
	CollectedEffect->SetRelativeRotation(FRotator(0.0f, 180.0f, 0.0f));
}
