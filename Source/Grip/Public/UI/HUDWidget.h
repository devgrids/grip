/**
*
* General HUD widgets implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* All of the basic structures and classes for rendering the player HUD, including
* the single full-screen HUD that encompasses all of the players for split-screen.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "blueprint/userwidget.h"
#include "vehicle/flippablevehicle.h"
#include "hudwidget.generated.h"

class USlateBrushAsset;
class UCanvasPanel;

/**
* A structure describing ignition properties for initializing a player HUD.
***********************************************************************************/

USTRUCT()
struct FHUDPanelIgnition
{
	GENERATED_USTRUCT_BODY()

public:

	FHUDPanelIgnition() = default;

	FHUDPanelIgnition(UUserWidget* widget, float startTime)
		: StartTime(startTime)
		, Widget(widget)
	{ }

	// The timer for this widget.
	float Timer = 0.0f;

	// The start time for when the widget will be ignited.
	float StartTime = 0.0f;

	// The widget to be ignited.
	UPROPERTY(Transient)
		UUserWidget* Widget = nullptr;
};

/**
* A structure describing a race position for rendering on the player HUD.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FHUDRacePosition
{
	GENERATED_USTRUCT_BODY()

public:

	// The relative distance of this vehicle to the player owning this HUD.
	UPROPERTY(BlueprintReadOnly)
		float RelativeWidgetDistance = 0.0f;

	// Is the vehicle a rival?
	UPROPERTY(BlueprintReadOnly)
		bool IsRival = false;

	// Is the vehicle using a Disruptor?
	UPROPERTY(BlueprintReadOnly)
		bool IsUsingDisruptor = false;

	// The color of the vehicle indicator.
	UPROPERTY(BlueprintReadOnly)
		FLinearColor Colour = FLinearColor::Black;
};

/**
* HUD widget component to provide an individual element of a player HUD.
***********************************************************************************/

UCLASS()
class GRIP_API UHUDWidgetComponent : public UUserWidget
{
	GENERATED_BODY()

public:

	// Construct a UHUDWidgetComponent.
	UHUDWidgetComponent(const FObjectInitializer& objectInitializer);

	// Cached reference to the game state.
	UPROPERTY(Transient, BlueprintReadOnly)
		UGlobalGameState* GameState = nullptr;

	// Cached reference to the play game mode.
	UPROPERTY(Transient, BlueprintReadOnly)
		APlayGameMode* PlayGameMode = nullptr;

	// Use ignition to spark the component to life a moment after its parent being switched on.
	UPROPERTY(EditAnywhere, BlueprintReadOnly)
		bool UseIgnition;

	// This component used in online games only.
	UPROPERTY(EditAnywhere, BlueprintReadOnly)
		bool OnlineOnly = false;

	// This component used in online spectator games only.
	UPROPERTY(EditAnywhere, BlueprintReadOnly)
		bool SpectatorOnly = false;

	// This component restricted to the listed driving modes?
	UPROPERTY(EditAnywhere, BlueprintReadOnly)
		bool RestrictDrivingModes = true;

	// Which driving modes this component is used for.
	UPROPERTY(EditAnywhere, BlueprintReadOnly, meta = (EditCondition = "RestrictDrivingModes"))
		TArray<EDrivingMode> DrivingModes;

	// Set the owning local player.
	UFUNCTION(BlueprintCallable, Category = HUD)
		void SetOwningLocalPlayer(ULocalPlayer* localPlayer)
	{ UUserWidget::SetOwningLocalPlayer(localPlayer); APlayerController* owningPlayer = GetOwningPlayer(); OwningVehicle = ((owningPlayer == nullptr) ? nullptr : Cast<ABaseVehicle>(owningPlayer->GetPawn())); }

	// Get the owning vehicle.
	UFUNCTION(BlueprintCallable, Category = HUD)
		ABaseVehicle* GetOwningVehicle() const
	{ return OwningVehicle; }

	// Get the target vehicle, the vehicle the viewport rendering this HUD is focused upon.
	UFUNCTION(BlueprintCallable, Category = HUD)
		ABaseVehicle* GetTargetVehicle() const
	{ return (OwningVehicle == nullptr) ? nullptr : OwningVehicle->CameraTarget(); }

	// Get the current driving mode.
	UFUNCTION(BlueprintCallable, Category = HUD)
		EDrivingMode GetDrivingMode() const
	{ return ((GameState == nullptr) ? EDrivingMode::Race : GameState->GamePlaySetup.DrivingMode); }

	// Get the label text for the player points.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetPointsLabelText() const
	{ return ((OwningVehicle == nullptr) ? FText::FromString("1UP") : FText::FromString(FString::Printf(TEXT("%dUP"), OwningVehicle->LocalPlayerIndex + 1))); }

	// Get the speedo text.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetSpeedoText() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return ((vehicle == nullptr) ? FText::FromString("000") : FText::FromString(vehicle->GetFormattedSpeedKPH(0))); }

	// Get the speedo text for the first portion of the rendering.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetSpeedoText1() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return ((vehicle == nullptr) ? FText::FromString("000") : FText::FromString(vehicle->GetFormattedSpeedKPH(0))); }

	// Get the speedo text for the second portion of the rendering.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetSpeedoText2() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return ((vehicle == nullptr) ? FText::FromString("000") : FText::FromString(vehicle->GetFormattedSpeedKPH(1))); }

	// Get the speed measurement text.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetKPHText() const;

	// Get the left text for the player position.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetPositionTextLeft() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return ((vehicle == nullptr || (vehicle->GetRaceState().LapNumber < 0 && GameState->GamePlaySetup.DrivingMode != EDrivingMode::Elimination)) ? FText::FromString("") : FText::FromString(FString::Printf(TEXT("%d"), ((GameState->IsGameModeRanked() == true) ? vehicle->GetRaceState().RaceRank + 1 : vehicle->GetRaceState().RacePosition + 1)))); }

	// Get the right text for the player position.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetPositionTextRight() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return ((vehicle == nullptr) ? FText::FromString("") : FText::FromString(FString::Printf(TEXT("/%d"), PlayGameMode->GetNumOpponentsLeft()))); }

	// Get the left text for the player rank.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetRankTextLeft() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return ((vehicle == nullptr) ? FText::FromString("") : FText::FromString(FString::Printf(TEXT("%d"), vehicle->GetRaceState().RaceRank + 1))); }

	// Get the left text for the player lap.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetLapTextLeft() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return ((vehicle == nullptr || vehicle->GetRaceState().LapNumber < 0) ? FText::FromString("") : FText::FromString(FString::Printf(TEXT("%d"), vehicle->GetRaceState().LapNumber + 1))); }

	// Get the right text for the player lap.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetLapTextRight() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return ((vehicle == nullptr) ? FText::FromString("") : FText::FromString(FString::Printf(TEXT("/%d"), (int32)GameState->GeneralOptions.NumberOfLaps))); }

	// Get the text for the player points.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetPointsText() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return ((vehicle == nullptr) ? FText::FromString("") : FText::FromString(FString::Printf(TEXT("%d"), vehicle->GetRaceState().NumInGamePoints))); }

	// Get the text for the player kills.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetKillsText() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return ((vehicle == nullptr) ? FText::FromString("") : FText::FromString(FString::Printf(TEXT("%d"), (int32)vehicle->GetRaceState().NumKills))); }

	// Get the text for the player deaths.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetDeathsText() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return ((vehicle == nullptr) ? FText::FromString("") : FText::FromString(FString::Printf(TEXT("/%d"), (int32)vehicle->GetRaceState().NumDeaths))); }

	// Get the text for the player damage.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetDamageText() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return ((vehicle == nullptr) ? FText::FromString("") : FText::FromString(FString::Printf(TEXT("%d%%"), (int32)((float)(vehicle->GetRaceState().MaxHitPoints - vehicle->GetRaceState().HitPoints) / (float)vehicle->GetRaceState().MaxHitPoints * 100)))); }

	// Get the text for the elimination timer.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetEliminationTimerText() const
	{ return (PlayGameMode->GetEliminationTimer() >= 0.0f) ? FText::FromString(FString::Printf(TEXT("%02d"), FMath::CeilToInt(GRIP_ELIMINATION_SECONDS - PlayGameMode->GetEliminationTimer()))) : FText::FromString("--"); }

	// Get the text for the elimination percentage.
	UFUNCTION(BlueprintCallable, Category = HUD)
		float GetEliminationPercent() const
	{ float timer = PlayGameMode->GetEliminationTimer() - (GRIP_ELIMINATION_SECONDS - GRIP_ELIMINATION_WARNING_SECONDS); return ((timer >= 0.0f) ? timer / GRIP_ELIMINATION_WARNING_SECONDS : 0.0f); }

	// Get the color for the elimination text.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FLinearColor GetEliminationColour() const
	{ return ((PlayGameMode->GetNumOpponentsLeft() - 1 == OwningVehicle->GetRaceState().RacePosition) ? FLinearColor(1.0f, 0.0f, 0.0f, 1.0f) : FLinearColor(0.0f, 1.0f, 0.0f, 1.0f)); }

	// Get the gear bar percentage.
	UFUNCTION(BlueprintCallable, Category = HUD)
		float GetGearBarPercent() const
	{ return FMath::RoundToFloat(GetTargetVehicle()->GearPosition() * 10.0f) / 10.0f; }

	// Get the race time of the vehicle.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetRaceTimeText() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return ((vehicle == nullptr) ? FText::FromString("") : FText::FromString(vehicle->GetFormattedRaceTime())); }

	// Get the number of seconds left in a race
	UFUNCTION(BlueprintCallable, Category = HUD)
		float GetRaceTimeRemaining() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return vehicle == nullptr ? 0.0f : FMath::Max(0.0f, ((GameState->GeneralOptions.MaximumGameTime * 60.f) - vehicle->GetRaceState().RaceTime)); }

	// Get the best lap time of the vehicle.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetBestLapTimeText() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return ((vehicle == nullptr) ? FText::FromString("") : FText::FromString(vehicle->GetFormattedBestLapTime())); }

	// Get the lap time of the vehicle.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetLapTimeText() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return ((vehicle == nullptr) ? FText::FromString("") : ((vehicle->GetRaceState().LapNumber >= 1 && vehicle->GetRaceState().LapTime < 5) ? FText::FromString(vehicle->GetFormattedLastLapTime()) : FText::FromString(vehicle->GetFormattedLapTime()))); }

	// Get the lap time opacity of the vehicle.
	UFUNCTION(BlueprintCallable, Category = HUD)
		float GetLapTimeOpacity() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return ((vehicle == nullptr) ? 1.0f : ((vehicle->GetRaceState().LapNumber >= 1 && vehicle->GetRaceState().LapTime < 5) ? PlayGameMode->GetFlashingOpacity() : 1.0f)); }

	// Get HUD opacity of the vehicle.
	UFUNCTION(BlueprintCallable, Category = HUD)
		float GetHUDOpacity(bool flashing, bool rapid = false) const
	{ return PlayGameMode->GetHUDScale() * ((flashing == true) ? PlayGameMode->GetFlashingOpacity(rapid) : 1.0f); }

	// Get the flashing opacity.
	UFUNCTION(BlueprintCallable, Category = HUD)
		float GetFlashingOpacity(bool flashing, bool rapid = false) const
	{ return ((flashing == true) ? PlayGameMode->GetFlashingOpacity(rapid) : 1.0f); }

	// Get the full color for rendering including the flashing opacity.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FLinearColor GetFullColour(bool flashing, bool rapid = false) const
	{ return FLinearColor(1.0f, 1.0f, 1.0f, ((flashing == true) ? PlayGameMode->GetFlashingOpacity(rapid) : 1.0f)); }

	// Get the lap time of the vehicle.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetCountDownText() const
	{ return PlayGameMode->GetCountDownTime(); }

	// Get the title text for the event outro.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetOutroTitleText() const;

	// Get the player placement text for the event outro.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetOutroPlacementText() const;

	// Get the position text for a placement in an event.
	UFUNCTION(BlueprintCallable, Category = HUD)
		static FText GetEventPositionText(int32 position, bool oneBased);

	// Get the comment text for the event outro.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetOutroCommentText() const;

	// Get the lap time of the vehicle.
	UFUNCTION(BlueprintCallable, Category = Vehicle)
		FLinearColor GetCountDownColour() const
	{ return FLinearColor(1.0f, 1.0f, 1.0f, PlayGameMode->GetCountdownOpacity()); }

	// Get the KPH/MPH visibility.
	UFUNCTION(BlueprintCallable, Category = HUD)
		ESlateVisibility GetKPHMPHVisibility() const
	{ return (GameState->GeneralOptions.SpeedUnit != ESpeedDisplayUnit::MACH) ? ESlateVisibility::HitTestInvisible : ESlateVisibility::Collapsed; }

	// Get the MACH visibility.
	UFUNCTION(BlueprintCallable, Category = HUD)
		ESlateVisibility GetMACHVisibility() const
	{ return (GameState->GeneralOptions.SpeedUnit == ESpeedDisplayUnit::MACH) ? ESlateVisibility::HitTestInvisible : ESlateVisibility::Collapsed; }

	// Get the countdown visibility.
	UFUNCTION(BlueprintCallable, Category = HUD)
		ESlateVisibility GetCountDownVisibility() const
	{ return (PlayGameMode->GetCountdownOpacity() > 0.0f) ? ESlateVisibility::HitTestInvisible : ESlateVisibility::Collapsed; }

	// Get the color of the wrong way indicator.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FLinearColor GetWrongWayColour() const
	{ return FLinearColor(1.0f, 1.0f, 1.0f, GetTargetVehicle()->GetWrongWayAlpha()); }

	// Get the color for the missile warning indicator.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FLinearColor GetMissileWarningColour() const
	{ return FLinearColor(1.0f, 0.0f, 0.0f, GetTargetVehicle()->GetMissileWarningAmount()); }

	// Get the color for the missile warning indicator.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FLinearColor GetAssistedMissileWarningColour() const;

	// Get the damage color.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FLinearColor GetDamageColour() const;

	// Get the visibility of the event playing panel.
	UFUNCTION(BlueprintCallable, Category = HUD)
		ESlateVisibility GetEventPlayingVisibility() const;

	// Get the visibility of the event intro panel.
	UFUNCTION(BlueprintCallable, Category = HUD)
		ESlateVisibility GetEventIntroVisibility() const;

	// Get the visibility of the event outro panel.
	UFUNCTION(BlueprintCallable, Category = HUD)
		ESlateVisibility GetEventOutroVisibility() const;

	// Get the visibility of the event summary panel.
	UFUNCTION(BlueprintCallable, Category = HUD)
		ESlateVisibility GetEventSummaryVisibility() const;

	// Get the visibility of the turn left indicator.
	UFUNCTION(BlueprintCallable, Category = HUD)
		ESlateVisibility VehicleShouldTurnLeft() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return ((vehicle != nullptr) ? (((GameState->IsTrackMirrored() == false) ? vehicle->ShouldTurnLeft() == true : vehicle->ShouldTurnRight() == true) ? ESlateVisibility::HitTestInvisible : ESlateVisibility::Collapsed) : ESlateVisibility::Collapsed); }

	// Get the visibility of the turn right indicator.
	UFUNCTION(BlueprintCallable, Category = HUD)
		ESlateVisibility VehicleShouldTurnRight() const
	{ ABaseVehicle* vehicle = GetTargetVehicle(); return ((vehicle != nullptr) ? (((GameState->IsTrackMirrored() == true) ? vehicle->ShouldTurnLeft() == true : vehicle->ShouldTurnRight() == true) ? ESlateVisibility::HitTestInvisible : ESlateVisibility::Collapsed) : ESlateVisibility::Collapsed); }

	// Get a pickup type for a slot index.
	UFUNCTION(BlueprintCallable, Category = HUD)
		EPickupType GetPickupType(int32 slotIndex) const;

	// Get a pickup charge level for a slot index.
	UFUNCTION(BlueprintCallable, Category = HUD)
		float GetPickupChargeLevel(int32 slotIndex, bool genuineChargeLevel = false) const;

	// Get a pickup discharge level for a slot index.
	UFUNCTION(BlueprintCallable, Category = HUD)
		float GetPickupDischargeLevel(int32 slotIndex) const;

	// Get a pickup charge color for a slot index.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FLinearColor GetPickupChargeColour(int32 slotIndex) const;

	// Get launch charge level.
	UFUNCTION(BlueprintCallable, Category = HUD)
		float GetLaunchChargeLevel() const;

	// Get the launch charge color.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FLinearColor GetLaunchChargeColour() const;

	// Get reset charge level.
	UFUNCTION(BlueprintCallable, Category = HUD)
		float GetResetChargeLevel() const;

	// Get the reset charge color.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FLinearColor GetResetChargeColour() const;

	// Get the color for pickup slot 1.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FLinearColor GetColourPickupSlot1() const
	{ return FLinearColor(1.0f, 1.0f, 1.0f, GetTargetVehicle()->GetPickupSlot1Alpha()); }

	// Get the color for pickup slot 2.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FLinearColor GetColourPickupSlot2() const
	{ return FLinearColor(1.0f, 1.0f, 1.0f, GetTargetVehicle()->GetPickupSlot2Alpha()); }

	// Get the scale for pickup slot 1.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FVector2D GetScalePickupSlot1() const
	{ float scale = (GetTargetVehicle() == nullptr) ? 0.0f : GetTargetVehicle()->GetPickupSlot1Scale(); return FVector2D(scale, scale); }

	// Get the scale for pickup slot 2.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FVector2D GetScalePickupSlot2() const
	{ float scale = (GetTargetVehicle() == nullptr) ? 0.0f : GetTargetVehicle()->GetPickupSlot2Scale(); return FVector2D(scale, scale); }

	// Is a particular weapon of interest to the HUD?
	UFUNCTION(BlueprintCallable, Category = HUD)
		bool IsWeaponEventOfInterest(int32 index) const;

	// Get the left-hand text for a weapon event.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetWeaponEventLeftText(int32 index) const;

	// Get the right-hand text for a weapon event.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetWeaponEventRightText(int32 index) const;

	// Get the left-hand color for a weapon event.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FSlateColor GetWeaponEventLeftColour(int32 index) const;

	// Get the right-hand color for a weapon event.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FSlateColor GetWeaponEventRightColour(int32 index) const;

	// Get the image index for a weapon event.
	UFUNCTION(BlueprintCallable, Category = HUD)
		int32 GetWeaponEventImageIndex(int32 index) const;

	// Get the opacity for a weapon event.
	UFUNCTION(BlueprintCallable, Category = HUD)
		static float GetWeaponEventOpacity(float timer, float maxTime)
	{ if (timer >= maxTime) return 0.0f; if (timer < (maxTime - 0.25f)) return 1.0f; return FMath::Sin((maxTime - timer) * 2.0f * PI); }

	// Get the text for the leading players distance ahead of the player.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetRacePositionScaleForeText() const
	{ return GetRacePositionScaleText(GetRacePositionScale()); }

	// Get the text for the trailing players distance behind the player.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetRacePositionScaleRearText() const
	{ return GetRacePositionScaleText(-GetRacePositionScale()); }

	// Get the text describing a race distance.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FText GetRacePositionScaleText(int32 distance) const;

	// Get the scale for rendering the race positions for the players.
	UFUNCTION(BlueprintCallable, Category = HUD)
		int32 GetRacePositionScale() const;

	// Get all of the race positions for all of the players.
	UFUNCTION(BlueprintCallable, Category = HUD)
		TArray<FHUDRacePosition> GetRacePositions(float centre, float length) const;

	// Get the angle of the ground from the perspective of the camera.
	UFUNCTION(BlueprintCallable, Category = HUD)
		float GetCameraGroundAngle() const;

	// Get the progress through the event between 0 and 1.
	UFUNCTION(BlueprintCallable, Category = HUD)
		float GetEventProgress() const;

	// Draw the vehicle diagnostics charts.
	UFUNCTION(BlueprintCallable, Category = HUD)
		static void DrawVehicleDiagnostics(UPARAM(ref) FPaintContext& context, USlateBrushAsset* brush, const ABaseVehicle* vehicle);

	UFUNCTION(BlueprintImplementableEvent, Category = HUD)
		void SetupHUDWidget(EDrivingMode drivingMode);

	// Let the HUD know what launch control was achieved.
	UFUNCTION(BlueprintImplementableEvent, Category = HUD)
		void ShowLaunchControl(int32 control);

	// Ignite the widget.
	void Ignite()
	{ Ignited = true; }

	// Record all of the widgets that can be ignited within a particular panel.
	void AddIgnitionWidgets(UPanelWidget* mainPanel);

	// Update the ignition sequences for the all the widgets on this particular panel.
	void UpdateIgnition(float deltaSeconds)
	{ if (Ignited == true) { for (FHUDPanelIgnition& widget : IgnitionWidgets) widget.Timer += deltaSeconds; } RefreshIgnitionWidgets(); }

	// Refresh all of the widgets that can be ignited.
	void RefreshIgnitionWidgets();

protected:

	// Post initialize the properties of the HUD widget component.
	virtual void PostInitProperties() override;

	// Begin destructing the HUD widget component.
	virtual void BeginDestroy() override;

	float GetPanelOpacity(FHUDPanelIgnition& panel)
	{ if (panel.Timer < panel.StartTime) return 0.15f; else return 1.0f; }

	// Refresh all of the widgets that can be ignited for a particular panel.
	void RefreshIgnitionWidgets(FHUDPanelIgnition& ignition, UPanelWidget* mainPanel);

	// Draw a vehicle diagnostics chart.
	static void DrawVehicleDiagnosticsChart(UPARAM(ref) FPaintContext& context, USlateBrushAsset* brush, const ABaseVehicle* vehicle, float x, float y, float width, float height, const FTimedFloatList& list, float scale, bool angles, bool calculateDifference, const FString& title);

	// The owning vehicle of this HUD component.
	// Naked pointer for speed, the vehicle is guaranteed to be valid for the lifetime of this widget.
	ABaseVehicle* OwningVehicle = nullptr;

	// Has this panel been ignited?
	bool Ignited = false;

private:

	// The widgets in this panel that require ignition.
	TArray<FHUDPanelIgnition> IgnitionWidgets;
};

/**
* A HUD widget for a single player.
***********************************************************************************/

UCLASS()
class GRIP_API UHUDWidget : public UHUDWidgetComponent
{
	GENERATED_BODY()

public:

	// Construct a UHUDWidget.
	UHUDWidget(const FObjectInitializer& objectInitializer);

	// Setup the HUD widget for a particular player.
	UFUNCTION(BlueprintCallable, Category = HUD)
		void SetupForPlayer(ULocalPlayer* localPlayer);

	// Setup the components of the HUD widget.
	UFUNCTION(BlueprintCallable, Category = HUD)
		void SetupComponents(UCanvasPanel* mainPanel) const;

	// Update the HUD widget.
	UFUNCTION(BlueprintCallable, Category = HUD)
		void Update(float deltaSeconds);

	// Setup the HUD widget for the particular game mode.
	UFUNCTION(BlueprintImplementableEvent, BlueprintCallable, Category = HUD)
		void SetupForGameMode(ULocalPlayer* localPlayer);

	// Show a status message.
	UFUNCTION(BlueprintImplementableEvent, BlueprintCallable, Category = HUD)
		void ShowStatusMessage(const FStatusMessage& message, bool queue);

	// Get the draw size for the widget.
	UFUNCTION(BlueprintCallable, Category = HUD)
		FVector2D GetDrawSize() const
	{ return Size; }

	// Set the draw size for the widget.
	UFUNCTION(BlueprintCallable, Category = HUD)
		void SetDrawSize(const FVector2D& size)
	{ Size = size; }

	// Signal to the HUD widget blueprint that a lap has been completed, and whether we're starting the final lap.
	UFUNCTION(BlueprintImplementableEvent, Category = HUD)
		void LapCompleted(bool startFinalLap);

	// Tick the HUD widget.
	virtual void NativeTick(const FGeometry& MyGeometry, float InDeltaTime) override;

	// Ignite the HUD and have the elements flicker on over time.
	void Ignite();

private:

	// The draw size of the widget.
	FVector2D Size = FVector2D::ZeroVector;

	// Has this HUD been ignited?
	bool Ignited = false;

	// The last lap that was signaled as completed.
	int32 LastLapCompleted = 0;
};

/**
* A single HUD widget to cover the entirety of the screen and provide individual
* player HUDs internally for split-screen.
***********************************************************************************/

UCLASS()
class GRIP_API USingleHUDWidget : public UUserWidget
{
	GENERATED_BODY()

public:

	// Hookup the HUD for a player.
	UFUNCTION(BlueprintImplementableEvent, BlueprintCallable, Category = HUD)
		UHUDWidget* HookupPlayerHUD(FVector2D canvasSize, FVector2D canvasOrigin);

	// Show and error message to the player.
	UFUNCTION(BlueprintImplementableEvent, BlueprintCallable, Category = HUD)
		void ShowErrorMessage(const FText& title, const FText& description, bool flashing, float time);

	// Clear any error messages shown to the player.
	UFUNCTION(BlueprintImplementableEvent, BlueprintCallable, Category = HUD)
		void ClearErrorMessage();

	// Is an error message currently visible to the player?
	UFUNCTION(BlueprintImplementableEvent, BlueprintCallable, Category = HUD)
		bool IsErrorMessageVisible();
};
