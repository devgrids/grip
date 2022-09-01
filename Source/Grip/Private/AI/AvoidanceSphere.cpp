/**
*
* Avoidance sphere implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* A sphere for describing a volume of space to be avoided.
*
***********************************************************************************/

#include "ai/avoidancesphere.h"
#include "gamemodes/playgamemode.h"

/**
* Construct an avoidance sphere.
***********************************************************************************/

AAvoidanceSphere::AAvoidanceSphere()
{
	SphericalVolume = CreateDefaultSubobject<USphereComponent>(TEXT("SphericalVolume"));

	SphericalVolume->ShapeColor = FColor::Blue;

	SetRootComponent(SphericalVolume);

	SphericalVolume->SetSphereRadius(500.0f);
	SphericalVolume->SetCollisionEnabled(ECollisionEnabled::Type::NoCollision);
	SphericalVolume->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
	SphericalVolume->SetMobility(EComponentMobility::Static);
}
