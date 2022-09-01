/**
*
* Common types, like enums and structs, that can be used anywhere and don't
* depend on other types in other headers.
*
* Original author: Chris Wood.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
***********************************************************************************/

#pragma once

#include "commontypes.generated.h"

/**
* The pickup types to be used for pickup pads.
***********************************************************************************/

UENUM(BlueprintType)
enum class EPickupType : uint8
{
	None,
	Random			UMETA(DisplayName = "Random"),
	Shield			UMETA(DisplayName = "Painkiller"),
	TurboBoost		UMETA(DisplayName = "Firestorm"),
	HomingMissile	UMETA(DisplayName = "Scorpion"),
	GatlingGun		UMETA(DisplayName = "Raptor"),
	Num				UMETA(Hidden)
};

/**
* The types, or phylum, of the vehicles.
***********************************************************************************/

UENUM(BlueprintType)
enum class EVehicleTypes : uint8
{
	None,
	Classic,
	Antigravity,
	Both
};

/**
* Which unit to use for displaying speed.
***********************************************************************************/

UENUM(BlueprintType)
enum class ESpeedDisplayUnit : uint8
{
	KPH,
	MPH,
	MACH
};

/**
* Which kind of game is being played?
***********************************************************************************/

UENUM(BlueprintType)
enum class EGameType : uint8
{
	None,

	// Single player event
	SinglePlayerEvent,

	// Split screen event
	SplitScreenEvent
};

/**
* How to show the player name tags, if at all.
***********************************************************************************/

UENUM(BlueprintType)
enum class EShowPlayerNameTags : uint8
{
	None,
	Rivals,
	All
};

/**
* Which kind of driving should be performed?
***********************************************************************************/

UENUM(BlueprintType)
enum class EDrivingMode : uint8
{
	None,
	Race				UMETA(DisplayName = "Classic Race"),
	Elimination			UMETA(DisplayName = "Elimination Race"),
	Num					UMETA(Hidden)
};

/**
* The anti-aliasing modes.
***********************************************************************************/

UENUM(BlueprintType)
enum class EAntiAliasingMode : uint8
{
	None,
	FXAA,
	TemporalAA
};

/**
* An off / on switch.
***********************************************************************************/

UENUM(BlueprintType)
enum class EOffOnSwitch : uint8
{
	Off,
	On
};

/**
* An off / on / random switch.
***********************************************************************************/

UENUM(BlueprintType)
enum class EOffOnRandomSwitch : uint8
{
	Off,
	On,
	Random
};

/**
* Quality levels.
***********************************************************************************/

UENUM(BlueprintType)
enum class EQualityLevel : uint8
{
	Off,
	Low,
	Medium,
	High,
	Epic
};

/**
* The split-screen layouts.
***********************************************************************************/

UENUM(BlueprintType)
enum class ESplitScreenLayout : uint8
{
	TwoPlayerVertical,
	TwoPlayerHorizontal
};

/**
* The cinematic camera mode.
***********************************************************************************/

UENUM(BlueprintType)
enum class ECinematicCameraMode : uint8
{
	Off,
	SplineFollowingVehicle,
	SplineFollowingVictimVehicle,
	CameraPointVehicle,
	CameraPointVehicleToProjectile,
	CameraPointVehicleToGun,
	CameraPointVehicleToVehicle,
	StaticCamera,
	SpiritWorld,
	CustomOverride,
	Num
};
