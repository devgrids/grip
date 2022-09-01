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

#pragma once

#include "system/gameconfiguration.h"
#include "trackcheckpoint.generated.h"

/**
* Track checkpoint actor for determining progress around a track.
***********************************************************************************/

UCLASS(Abstract)
class GRIP_API ATrackCheckpoint : public AActor
{
	GENERATED_BODY()

public:

	// Construct a checkpoint.
	ATrackCheckpoint();

	// Use the planar size of the checkpoint rather than assuming infinite size.
	UPROPERTY(EditAnywhere, Category = Checkpoint)
		bool UseCheckpointSize = false;

	// The order number for correct traversal.
	UPROPERTY(EditAnywhere, Category = Checkpoint)
		int32 Order = 0;

	// The width of the checkpoint, applied to the scale of the Passing Volume.
	UPROPERTY(EditAnywhere, Category = Checkpoint)
		float Width = 50.0f;

	// The height of the checkpoint, applied to the scale of the Passing Volume.
	UPROPERTY(EditAnywhere, Category = Checkpoint)
		float Height = 50.0f;

	// The mesh to use to show the direction of the checkpoint in the Editor.
	UPROPERTY(EditAnywhere, Category = Checkpoint)
		UStaticMeshComponent* DirectionMesh = nullptr;

	// Box to describe the plane (X component is ignored) across which a vehicle must pass.
	UPROPERTY(EditAnywhere, Category = Checkpoint)
		UBoxComponent* PassingVolume = nullptr;

	// The distance this checkpoint is along the master racing spline.
	float DistanceAlongMasterRacingSpline = 0.0f;
};
