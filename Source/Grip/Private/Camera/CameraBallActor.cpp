/**
*
* Camera ball implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Actually, it's a cube rather than a ball as otherwise it rolls for far too long
* once it hit the ground. This is simply a physics object to which we can attach
* a camera and throw it into the world and let it do whatever it does.
*
* It's used when a vehicle is destroyed and when a vehicle hits a track camera when
* in cinematic camera mode.
*
***********************************************************************************/

#include "camera/cameraballactor.h"

/**
* Construct camera ball.
***********************************************************************************/

ACameraBallActor::ACameraBallActor()
{
	CollisionShape = CreateDefaultSubobject<UBoxComponent>(TEXT("CollisionShape"));

	CollisionShape->SetCollisionEnabled(ECollisionEnabled::Type::PhysicsOnly);
	CollisionShape->SetCollisionProfileName(FName("CameraBall"));
	CollisionShape->SetSimulatePhysics(true);
	CollisionShape->SetLinearDamping(0.5f);
	CollisionShape->SetAngularDamping(0.333f);

	SetRootComponent(CollisionShape);
}
