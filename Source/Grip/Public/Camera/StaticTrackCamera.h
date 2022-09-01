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

#pragma once

#include "system/gameconfiguration.h"
#include "ai/pursuitsplineactor.h"
#include "statictrackcamera.generated.h"

/**
* Track camera actor for placing camera views around tracks.
***********************************************************************************/

UCLASS(Blueprintable, ClassGroup = Camera, HideCategories = (Input, Physics, Rendering))
class GRIP_API AStaticTrackCamera : public AActor
{
	GENERATED_BODY()

public:

	AStaticTrackCamera();

	// The camera component to use for this track camera.
	UPROPERTY(EditAnywhere, Category = TrackCamera, meta = (AllowPrivateAccess = "true"))
		class UCameraComponent* Camera = nullptr;

	// The duration the camera is to be used for.
	UPROPERTY(EditAnywhere, Category = TrackCamera)
		float Duration = 4.0f;

	// The delay time in switching to this camera once vehicles have been detected coming towards it.
	UPROPERTY(EditAnywhere, Category = TrackCamera)
		float HookupDelay = 0.0f;

	// The number of vehicles that must be detected in proximity before the camera will be used.
	UPROPERTY(EditAnywhere, Category = TrackCamera)
		int32 NumberOfVehicles = 3;

	// Only detect vehicles on the closest pursuit spline to this camera.
	UPROPERTY(EditAnywhere, Category = TrackCamera)
		bool LinkToClosestPursuitSpline = false;

	// Is this camera indestructible and cannot be damaged?
	UPROPERTY(EditAnywhere, Category = TrackCamera)
		bool Indestructible = false;

	// Respond to a vehicle hitting the camera, often by throwing it off its mount and onto the track.
	UFUNCTION()
		void OnVehicleHit(class UPrimitiveComponent* hitComponent, class AActor* otherActor, class UPrimitiveComponent* otherComponent, int32 otherBodyIndex, bool fromSweep, const FHitResult& sweepResult);

private:

	// Collision box to detect vehicles impacting the camera.
	UPROPERTY(Transient)
		UBoxComponent* CollisionBox = nullptr;

	// The sound to use for impacting the camera.
	UPROPERTY(Transient)
		USoundCue* ImpactSound = nullptr;
};
