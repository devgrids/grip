/**
*
* Flippable vehicle implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* This is a simple override of the ABaseVehicle class where we just indicate that
* the vehicle is flippable. The ABaseVehicle class is non-flippable by default.
*
***********************************************************************************/

#pragma once

#include "Vehicle/BaseVehicle.h"
#include "FlippableVehicle.generated.h"

#pragma region MinimalVehicle

/**
* Flippable vehicle class, a simple customization of the ABaseVehicle class.
***********************************************************************************/

UCLASS(Abstract, Blueprintable, ClassGroup = Vehicle)
class GRIP_API AFlippableVehicle : public ABaseVehicle
{
	GENERATED_BODY()

public:

	// Is the vehicle flippable and has bidirectional wheels?
	virtual bool IsFlippable() override
	{ return true; }
};

#pragma endregion MinimalVehicle
