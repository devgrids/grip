/**
*
* Vehicle pickups implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Handle all of the pickups-related activity for the vehicle, mostly related to the
* two pickup slots that each vehicle has for two different pickups.
*
***********************************************************************************/

#include "vehicle/flippablevehicle.h"
#include "gamemodes/playgamemode.h"
#include "game/globalgamestate.h"
#include "pickups/speedpad.h"
#include "pickups/gatlinggun.h"
#include "pickups/turbo.h"
#include "pickups/shield.h"
#include "components/widgetcomponent.h"

DEFINE_LOG_CATEGORY_STATIC(GripLogPickups, Warning, All);

/**
* Give a particular pickup to a vehicle.
***********************************************************************************/

int32 ABaseVehicle::GivePickup(EPickupType type, int32 pickupSlot, bool fromTrack)
{
	return -1;
}

/**
* Is a pickup slot currently charging?
***********************************************************************************/

bool FPlayerPickupSlot::IsCharging(bool confirmed) const
{
	return ChargingState == EPickupSlotChargingState::Charging && (confirmed == false || HookTimer >= ABaseVehicle::PickupHookTime);
}
