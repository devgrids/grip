/**
*
* Static track camera implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Track cameras are placed around the track to show views of the race after it has
* finished when in cinematic camera mode, or when in attract mode from the main
* menu.
*
***********************************************************************************/

#include "camera/statictrackcamera.h"
#include "ai/pursuitsplineactor.h"
#include "system/worldfilter.h"
#include "vehicle/flippablevehicle.h"
#include "gamemodes/playgamemode.h"

/**
* Construct a AStaticTrackCamera.
***********************************************************************************/

AStaticTrackCamera::AStaticTrackCamera()
{
	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->bConstrainAspectRatio = false;
	Camera->AspectRatio = 1.777778f;
	Camera->PostProcessBlendWeight = 1.0f;
	Camera->SetFieldOfView(30.0f);
	Camera->SetMobility(EComponentMobility::Static);

	SetRootComponent(Camera);
}

/**
* Respond to a vehicle hitting the camera, often by throwing it off its mount and
* onto the track.
***********************************************************************************/

void AStaticTrackCamera::OnVehicleHit(class UPrimitiveComponent* hitComponent, class AActor* otherActor, class UPrimitiveComponent* otherComponent, int32 otherBodyIndex, bool fromSweep, const FHitResult& sweepResult)
{
}
