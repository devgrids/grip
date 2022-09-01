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

#pragma once

#include "system/gameconfiguration.h"
#include "cameraballactor.generated.h"

/**
* A camera ball actor, a physics object to which we can attach a camera and throw
* it into the world
***********************************************************************************/

UCLASS(ClassGroup = Camera)
class GRIP_API ACameraBallActor : public AActor
{
	GENERATED_BODY()

public:

	// Construct camera ball.
	ACameraBallActor();

	// Sphere for overlay detection.
	UPROPERTY(EditDefaultsOnly, Category = Default)
		UBoxComponent* CollisionShape = nullptr;
};
