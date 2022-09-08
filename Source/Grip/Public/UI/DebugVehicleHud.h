/**
*
* Vehicle debugging HUD.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "debughud.h"
#include "debugvehiclehud.generated.h"

/**
* The general debugging HUD for the vehicle.
***********************************************************************************/

UCLASS()
class ADebugVehicleHUD : public ADebugHUD
{
	GENERATED_BODY()

public:

#pragma region VehicleContactSensors

	// Draw the HUD.
	virtual void DrawHUD() override;

#pragma endregion VehicleContactSensors

};
