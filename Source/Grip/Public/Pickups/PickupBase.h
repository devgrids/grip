/**
*
* Base pickup implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Basic pickup type, inherited by all of the different pickups in the game.
*
***********************************************************************************/

#pragma once

#include "pickups/pickup.h"
#include "pickupbase.generated.h"

class ABaseVehicle;
class UGlobalGameState;
class APlayGameMode;

/**
* Activation type for handling the activation of pickups.
***********************************************************************************/

enum class EPickupActivation : uint8
{
	None,

	// The pickup input has just been pressed down
	Pressed,

	// The pickup input has just been released
	Released
};

/**
* Basic pickup type, inherited by all of the different pickups in the game.
***********************************************************************************/

UCLASS(Abstract, ClassGroup = Pickups)
class GRIP_API APickupBase : public AActor
{
	GENERATED_BODY()

public:

	// Get the launch vehicle for this pickup.
	UFUNCTION(BlueprintCallable, Category = Pickup)
		ABaseVehicle* GetLaunchVehicle() const
	{ check(LaunchVehicleIsValid() == true); return LaunchVehicle; }

	// Is the launch vehicle valid for this pickup?
	bool LaunchVehicleIsValid() const
	{ return (LaunchVehicle != nullptr); }

protected:

	// Naked pointer to game state for performance reasons.
	UPROPERTY(Transient)
		UGlobalGameState* GameState = nullptr;

	// Naked pointer to play game mode for performance reasons.
	UPROPERTY(Transient)
		APlayGameMode* PlayGameMode = nullptr;

	// What type is this pickup?
	EPickupType PickupType = EPickupType::None;

	// The launch vehicle for this pickup.
	UPROPERTY(Transient)
		ABaseVehicle* LaunchVehicle = nullptr;
};
