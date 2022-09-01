/**
*
* Track checkpoint implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Track checkpoints are used to determine vehicle progress around a track. They're
* attached to the master racing spline so we know their positions along that spline
* and thus when a vehicle crossed their position. They can be set to have a size
* so vehicle need to physically pass through a 3D window to register that passing,
* but this generally isn't necessary - track position is normally enough.
*
***********************************************************************************/

#include "ai/trackcheckpoint.h"

/**
* Construct a checkpoint.
***********************************************************************************/

ATrackCheckpoint::ATrackCheckpoint()
{
	DirectionMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("DirectionMesh"));
	DirectionMesh->SetRelativeScale3D(FVector(1.0f, 1.0f, 1.0f));
	DirectionMesh->SetHiddenInGame(true);

	SetRootComponent(DirectionMesh);

	PassingVolume = CreateDefaultSubobject<UBoxComponent>(TEXT("PassingVolume"));
	PassingVolume->SetBoxExtent(FVector(50.0f, 50.0f, 50.0f));

	GRIP_ATTACH(PassingVolume, RootComponent, NAME_None);
}
