/**
*
* Vehicle HUD implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Some data structures used to help managed the HUD for a vehicle.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "vehiclehud.generated.h"

class ABaseVehicle;
class UPanelWidget;

#pragma region VehicleHUD

/**
* A warning source that will be used to color the player's peripheral vision.
***********************************************************************************/

enum class EHUDWarningSource : uint8
{
	None,
	Elimination,
	StandardPickup,
	CollectablePickup,
	HealthPickup,
	DoubleDamagePickup,
	DoubleDamage
};

/**
* A structure describing a status message to be displayed on the player's HUD.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FStatusMessage
{
	GENERATED_USTRUCT_BODY()

public:

	FStatusMessage() = default;

	FStatusMessage(const FText& message)
		: Message(message)
	{ }

	// The message to be displayed.
	UPROPERTY(Transient, BlueprintReadWrite)
		FText Message;

	// The time period it's to be displayed for.
	UPROPERTY(Transient, BlueprintReadOnly)
		float Time = 4.0f;

	// Is this message flashing?
	UPROPERTY(Transient, BlueprintReadOnly)
		bool Flashing = false;

	// Is this message animated?
	UPROPERTY(Transient, BlueprintReadOnly)
		bool Animated = true;
};

/**
* A structure describing a weapon target to be displayed on the player's HUD.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FHUDTarget
{
	GENERATED_USTRUCT_BODY()

public:

	FHUDTarget() = default;

	FHUDTarget(TWeakObjectPtr<AActor> target, bool primary)
		: Target(target)
		, Primary(primary)
	{ }

	// Target actor that is the focus of the target.
	UPROPERTY(Transient, BlueprintReadOnly)
		TWeakObjectPtr<AActor> Target;

	// Timer associated with the timer.
	UPROPERTY(Transient, BlueprintReadOnly)
		float TargetTimer = 0.0f;

	// Is this the primary target for the owning vehicle?
	UPROPERTY(Transient, BlueprintReadOnly)
		bool Primary = false;
};

/**
* A structure used to manage the player's HUD as a whole.
***********************************************************************************/

struct FVehicleHUD
{
public:

	// Get the current missile target for a pickup slot.
	FHUDTarget* GetCurrentMissileTarget(int32 pickupSlot)
	{ return (CurrentMissileTarget[pickupSlot] < 0) ? nullptr : &PickupTargets[pickupSlot][CurrentMissileTarget[pickupSlot]]; }

	// Get the current missile target actor for a pickup slot.
	AActor* GetCurrentMissileTargetActor(int32 pickupSlot)
	{ return (CurrentMissileTarget[pickupSlot] < 0) ? nullptr : PickupTargets[pickupSlot][CurrentMissileTarget[pickupSlot]].Target.Get(); }

	// Is the current missile target valid for a pickup slot?
	bool CurrentMissileTargetIsValid(int32 pickupSlot)
	{ return (CurrentMissileTarget[pickupSlot] < 0) ? false : GRIP_POINTER_VALID(PickupTargets[pickupSlot][CurrentMissileTarget[pickupSlot]].Target); }

	// Set the warning properties for the HUD.
	void Warning(EHUDWarningSource source, float timer, float alpha)
	{ if ((int32)WarningSource <= (int32)source) { WarningSource = source; WarningTimer = timer; WarningAlpha = alpha; } }

	// Has the HUD been hooked to the vehicle?
	bool HUDHooked = false;

	// The duration to shake the HUD for.
	float ShakeTime = 0.0f;

	// The timer used in shaking the HUD.
	float ShakeTimer = 0.0f;

	// The current magnitude of the HUD shaking.
	float ShakeMagnitude = 0.0f;

	// Timer used for displaying the wrong way message.
	float WrongWayTimer = 0.0f;

	// The ratio between 0 and 1 of how far the closest missile is this vehicle.
	float HomingMissileTime = 0.0f;

	// Timer used for managing the incoming missile audio indicator.
	float HomingMissileTimer = 0.0f;

	// The missile warning amount between 0 and 1, 1 being imminent.
	float MissileWarningAmount = 0.0f;

	// Timer used for managing the incoming missile indicator.
	float MissileWarningTimer = 0.0f;

	// The amount to use for the screen periphery.
	float WarningAmount = 0.0f;

	// The timer used for the warning on screen periphery.
	float WarningTimer = 0.0f;

	// The alpha used for the warning on screen periphery, often combined with WarningAmount.
	float WarningAlpha = 0.0f;

	// The source of the warning on screen periphery.
	EHUDWarningSource WarningSource = EHUDWarningSource::None;

	// Timer used in switching targets for the homing missile.
	float SwitchTargetTimer = 0.0f;

	// Has launch control been signaled to the player?
	bool LaunchControlShown = false;

	// Indices of the current missile target for each pickup slot.
	int32 CurrentMissileTarget[2] = { -1, -1 };

	// Location of the current missile target for each pickup slot.
	FVector TargetLocation[2];

	// Last location of the current missile target for each pickup slot.
	FVector LastTargetLocation[2];

	// The pickup targets for each pickup slot.
	TArray<FHUDTarget> PickupTargets[2];

	// The threat targets for each pickup slot.
	TArray<FHUDTarget> ThreatTargets;

	// The last targets for each pickup slot.
	TWeakObjectPtr<AActor> LastTarget[2];

	// The size of the HUD widget.
	FVector2D WidgetPositionSize = FVector2D(1.0f, 1.0f);

	// The scale of the HUD widget.
	FVector2D WidgetPositionScale = FVector2D(1.0f, 1.0f);

	// Audio to use for the homing missile indicator sound.
	static USoundCue* HomingMissileIndicatorSound;

	// Audio to use for the critical homing missile indicator sound.
	static USoundCue* HomingMissileIndicatorCriticalSound;

	// Audio to use for the pickup charged sound.
	static USoundCue* PickupChargedSound;

	// Audio to use for the pickup charging sound.
	static USoundCue* PickupChargingSound;

	// Audio to use for the pickup not-chargeable sound.
	static USoundCue* PickupNotChargeableSound;
};

#pragma endregion VehicleHUD
