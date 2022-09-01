/**
*
* Flippable spring arm implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Provides a spring arm for a camera that works well with flippable vehicles and
* contains a number of improvements over and above the standard engine spring arm.
* It doesn't care if the vehicle it's linked to isn't flippable, it doesn't matter.
*
* So the spring arm is used as a device to control where the camera sits behind a
* car when racing - showing the best view of the car to the player.
*
* It should always sit above and behind the car with respect to where the driving
* surface is. Defining "behind" is simple, but defining "above" is not quite so
* easy as the cars can flip over and the track can be upside down. Inevitably
* though, this will boil down to a world direction vector, which we can query
* directly from GetLaunchDirection on the vehicle.
*
* Positioning of the camera should be subject to smoothing as we don't want it
* violently moving from one frame to the next.
*
* The spring arm has several states of operation which we need to smoothly link to
* in order to avoid rough camera-work which is jarring to the player. Especially
* important here is the detection of the crashed state and our transition to and
* from it.
*
* We need to be more choosy about when transitioning back from airborne to normal
* states and be sure we really are in a normal state as erratic landings can give
* jarring camera-work at times.
*
* Crashed state means the vehicle is tumbling but in rough contact with the ground
* rather than being fully airborne (in which case tumbling isn't really an issue).
* It can also mean that the vehicle is jammed-up somewhere or close to a standing
* start but pointing in the wrong direction. It can also mean we're driving on a
* surface that is known to be invalid.
*
* We need camera-work here that handles all these situations well, and the
* transition out of it to normal driving can be delayed until we're definitely sure
* normal driving is being done, without any ill affect upon playability.
*
* Ensure all positional offsets / rotations are Schmitt-triggered and smoothed
* during the crash camera as it needs to be as reorienting and stable as possible
* for the player.
*
* So, prefer the race camera where possible.
* Airborne camera, following the velocity vector, when airborne.
* Crash camera, when the player is in trouble and in rough contact with the ground.
*
***********************************************************************************/

#include "camera/flippablespringarmcomponent.h"
#include "vehicle/flippablevehicle.h"
#include "system/mathhelpers.h"
#include "gamemodes/basegamemode.h"

const FName UFlippableSpringArmComponent::SocketName(TEXT("SpringEndpoint"));

/**
* Construct a flippable spring arm component.
***********************************************************************************/

UFlippableSpringArmComponent::UFlippableSpringArmComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;

	bAutoActivate = true;
	bTickInEditor = true;

	CameraOffsets.Emplace(FCameraOffset(-800.0f, 800.0f, 5.0f, 5.0f, 1.0f));
	CameraOffsets.Emplace(FCameraOffset(-600.0f, 600.0f, 5.0f, 5.0f, 1.0f));
	CameraOffsets.Emplace(FCameraOffset(-400.0f, 400.0f, 5.0f, 5.0f, 1.0f));
	CameraOffsets.Emplace(FCameraOffset(-200.0f, 200.0f, 5.0f, 5.0f, 0.0f));
}
