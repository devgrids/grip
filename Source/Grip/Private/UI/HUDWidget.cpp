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

#include "ui/hudwidget.h"
#include "blueprint/userwidget.h"
#include "components/canvaspanel.h"
#include "components/textwidgettypes.h"
#include "components/textblock.h"
#include "components/contentwidget.h"
#include "components/border.h"
#include "components/widgetcomponent.h"
#include "blueprint/widgetblueprintlibrary.h"
#include "gamemodes/playgamemode.h"

/**
* Construct a UHUDWidgetComponent.
***********************************************************************************/

UHUDWidgetComponent::UHUDWidgetComponent(const FObjectInitializer& objectInitializer)
	: Super(objectInitializer)
{ }

/**
* Post initialize the properties of the HUD widget component.
***********************************************************************************/

void UHUDWidgetComponent::PostInitProperties()
{
	Super::PostInitProperties();

	if (GetWorld() != nullptr)
	{
		PlayGameMode = APlayGameMode::Get(this);
		GameState = UGlobalGameState::GetGlobalGameState(GetWorld());

		APlayerController* owningPlayer = GetOwningPlayer();

		if (owningPlayer != nullptr)
		{
			OwningVehicle = Cast<ABaseVehicle>(owningPlayer->GetPawn());
		}
	}
}

/**
* Begin destructing the HUD widget component.
***********************************************************************************/

void UHUDWidgetComponent::BeginDestroy()
{
	Super::BeginDestroy();

	PlayGameMode = nullptr;
}

/**
* Get the visibility of the event summary panel.
***********************************************************************************/

ESlateVisibility UHUDWidgetComponent::GetEventSummaryVisibility() const
{
	// Force the scoreboard on if we're a spectator...

	ABaseVehicle* vehicle = GetOwningVehicle();

	if (vehicle != nullptr)
	{
		switch (GetDrivingMode())
		{
		case EDrivingMode::Race:
		case EDrivingMode::Elimination:
			if (vehicle->GetGameEndedClock() > APlayGameMode::RaceOutroTime)
			{
				return ESlateVisibility::HitTestInvisible;
			}
		}
	}

	return ESlateVisibility::Collapsed;
}

/**
* Get the visibility of the event intro panel.
***********************************************************************************/

ESlateVisibility UHUDWidgetComponent::GetEventIntroVisibility() const
{
	ABaseVehicle* vehicle = GetOwningVehicle();

	if (vehicle != nullptr)
	{
		if (GetEventSummaryVisibility() == ESlateVisibility::Collapsed)
		{
			return ESlateVisibility::HitTestInvisible;
		}
	}

	return ESlateVisibility::Collapsed;
}

/**
* Get the visibility of the event outro panel.
***********************************************************************************/

ESlateVisibility UHUDWidgetComponent::GetEventOutroVisibility() const
{
	ABaseVehicle* vehicle = GetOwningVehicle();

	if (vehicle != nullptr)
	{
		switch (GetDrivingMode())
		{
		case EDrivingMode::Race:
		case EDrivingMode::Elimination:
			if (vehicle->GetRaceState().PlayerCompletionState >= EPlayerCompletionState::Complete &&
				GetEventSummaryVisibility() == ESlateVisibility::Collapsed)
			{
				return ESlateVisibility::HitTestInvisible;
			}
		}
	}

	return ESlateVisibility::Collapsed;
}

/**
* Get the warning color for the missile warning indicator.
***********************************************************************************/

FLinearColor UHUDWidgetComponent::GetAssistedMissileWarningColour() const
{
	if (GetTargetVehicle()->GetMissileWarningAmount() < 0.25f)
	{
		return FLinearColor(1.0f, 1.0f, 1.0f, 1.0f);
	}
	else
	{
		return FLinearColor(1.0f, 0.0f, 0.0f, 1.0f);
	}
}

/**
* Get the damage color.
***********************************************************************************/

FLinearColor UHUDWidgetComponent::GetDamageColour() const
{
	ABaseVehicle* vehicle = GetTargetVehicle();

	if (vehicle != nullptr)
	{
		float damage = (float)vehicle->GetRaceState().HitPoints / (float)vehicle->GetRaceState().MaxHitPoints;

		if (damage < 0.25f)
		{
			return FLinearColor(1.0f, 0.0f, 0.0f, 1.0f);
		}
		else if (damage < 0.5f)
		{
			return FLinearColor(1.0f, 0.5f, 0.0f, 1.0f);
		}
		else if (damage < 0.75f)
		{
			return FLinearColor(1.0f, 1.0f, 0.0f, 1.0f);
		}
	}

	return FLinearColor(1.0f, 1.0f, 1.0f, 1.0f);
}

/**
* Get the visibility of the event playing panel.
***********************************************************************************/

ESlateVisibility UHUDWidgetComponent::GetEventPlayingVisibility() const
{
	ABaseVehicle* vehicle = GetOwningVehicle();

	if (vehicle != nullptr &&
		PlayGameMode->GetHUDScale() > 0.0f)
	{
		ESlateVisibility visible = (GetEventSummaryVisibility() != ESlateVisibility::Collapsed) ? ESlateVisibility::Collapsed : ESlateVisibility::HitTestInvisible;

		if (visible != ESlateVisibility::Collapsed)
		{
			visible = ESlateVisibility::HitTestInvisible;
		}

		if (visible != ESlateVisibility::Collapsed)
		{
			if (vehicle->GetRaceState().PlayerCompletionState >= EPlayerCompletionState::Complete)
			{
				visible = ESlateVisibility::Collapsed;
			}
		}

		return visible;
	}

	return ESlateVisibility::Collapsed;
}

/**
* Get the player placement text for the event outro.
***********************************************************************************/

FText UHUDWidgetComponent::GetOutroPlacementText() const
{
	ABaseVehicle* vehicle = GetOwningVehicle();

	if (vehicle != nullptr)
	{
		int32 position = vehicle->GetRaceState().RacePosition + 1;

		if (GameState->IsGameModeRanked() == true)
		{
			position = vehicle->GetRaceState().RaceRank + 1;
		}

		if (vehicle->GetRaceState().PlayerCompletionState == EPlayerCompletionState::Disqualified)
		{
			return NSLOCTEXT("GripScoreboard", "DNF", "DNF");
		}
		else
		{
			return GetEventPositionText(position, true);
		}
	}
	else
	{
		return FText::FromString("");
	}
}

/**
* Get the position text for a placement in an event.
***********************************************************************************/

FText UHUDWidgetComponent::GetEventPositionText(int32 position, bool oneBased)
{
	if (position < 0)
	{
		return NSLOCTEXT("GripScoreboard", "NA", "N/A");
	}
	else
	{
		if (oneBased == false)
		{
			position++;
		}

		switch (position)
		{
		case 1:
			return NSLOCTEXT("GripScoreboard", "1st", "1st");
		case 2:
			return NSLOCTEXT("GripScoreboard", "2nd", "2nd");
		case 3:
			return NSLOCTEXT("GripScoreboard", "3rd", "3rd");
		case 4:
			return NSLOCTEXT("GripScoreboard", "4th", "4th");
		case 5:
			return NSLOCTEXT("GripScoreboard", "5th", "5th");
		case 6:
			return NSLOCTEXT("GripScoreboard", "6th", "6th");
		case 7:
			return NSLOCTEXT("GripScoreboard", "7th", "7th");
		case 8:
			return NSLOCTEXT("GripScoreboard", "8th", "8th");
		case 9:
			return NSLOCTEXT("GripScoreboard", "9th", "9th");
		case 10:
			return NSLOCTEXT("GripScoreboard", "10th", "10th");
		}

		return FText::FromString("");
	}
}

/**
* Get the speed measurement text.
***********************************************************************************/

FText UHUDWidgetComponent::GetKPHText() const
{
	if (GameState == nullptr)
	{
		return FText::FromString("kph");
	}
	else
	{
		switch (GameState->GeneralOptions.SpeedUnit)
		{
		case ESpeedDisplayUnit::MPH:
			return NSLOCTEXT("GripScoreboard", "mph", "mph");
		case ESpeedDisplayUnit::KPH:
			return NSLOCTEXT("GripScoreboard", "kph", "kph");
		default:
			return NSLOCTEXT("GripScoreboard", "mach", "mach");
		}
	}
}

/**
* Get the title text for the event outro.
***********************************************************************************/

FText UHUDWidgetComponent::GetOutroTitleText() const
{
	ABaseVehicle* vehicle = GetOwningVehicle();

	if (vehicle != nullptr)
	{
		if (vehicle->GetRaceState().PlayerCompletionState == EPlayerCompletionState::Disqualified)
		{
			return NSLOCTEXT("GripScoreboard", "LostIt", "LOST IT");
		}
	}

	return NSLOCTEXT("GripScoreboard", "YouCame", "YOU CAME");
}

/**
* Get the comment text for the event outro.
***********************************************************************************/

FText UHUDWidgetComponent::GetOutroCommentText() const
{
	return FText::FromString("");
}

/**
* Get a pickup type for a slot index.
***********************************************************************************/

EPickupType UHUDWidgetComponent::GetPickupType(int32 slotIndex) const
{
	ABaseVehicle* vehicle = GetTargetVehicle();

	return (vehicle != nullptr) ? vehicle->GetPickupSlot(slotIndex).Type : EPickupType::None;
}

/**
* Get a pickup charge level for a slot index.
***********************************************************************************/

float UHUDWidgetComponent::GetPickupChargeLevel(int32 slotIndex, bool genuineChargeLevel) const
{
	ABaseVehicle* vehicle = GetTargetVehicle();

	if (vehicle != nullptr)
	{
		if (vehicle->GetPickupSlot(slotIndex).IsCharging(true) == true ||
			vehicle->GetPickupSlot(slotIndex).IsCharged() == true)
		{
			return vehicle->GetPickupSlot(slotIndex).GetChargeTimer();
		}

		if (genuineChargeLevel == false &&
			vehicle->GetRaceState().DoubleDamage > 0.0f &&
			vehicle->GetPickupSlot(slotIndex).IsEmpty() == false)
		{
			return 1.0f;
		}
	}

	return 0.0f;
}

/**
* Get a pickup discharge level for a slot index.
***********************************************************************************/

float UHUDWidgetComponent::GetPickupDischargeLevel(int32 slotIndex) const
{
	ABaseVehicle* vehicle = GetTargetVehicle();

	if (vehicle != nullptr)
	{
		if (vehicle->GetPickupSlot(slotIndex ^ 1).IsCharging(true) == true)
		{
			return vehicle->GetPickupSlot(slotIndex ^ 1).GetChargeTimer();
		}
	}

	return 0.0f;
}

/**
* Get a pickup charge color for a slot index.
***********************************************************************************/

FLinearColor UHUDWidgetComponent::GetPickupChargeColour(int32 slotIndex) const
{
	ABaseVehicle* vehicle = GetTargetVehicle();

	if (vehicle != nullptr)
	{
		if (vehicle->GetPickupSlot(slotIndex ^ 1).IsCharging(true) == true)
		{
			return FLinearColor(1.0f, 0.0f, 0.0f, GetFlashingOpacity(true, true));
		}

		float alpha = 0.0f;

		if (vehicle->GetRaceState().DoubleDamage > 0.0f)
		{
			if (vehicle->GetPickupSlot(slotIndex).IsActive() == true)
			{
				return FLinearColor(0.4f, 0.0f, 0.8f, GetFlashingOpacity(true, true));
			}
			else if (vehicle->GetPickupSlot(slotIndex).IsEmpty() == false)
			{
				return FLinearColor(0.4f, 0.0f, 0.8f, alpha);
			}
		}

		if (vehicle->GetPickupSlot(slotIndex).IsCharging(true) == true ||
			vehicle->GetPickupSlot(slotIndex).IsCharged() == true)
		{
			return vehicle->GetPickupSlot(slotIndex).GetChargeTimer() < 1.0f ? FLinearColor(1.0f, 0.0f, 0.0f, alpha) : FLinearColor(0.0f, 1.0f, 0.0f, alpha);
		}
	}

	return FLinearColor(1.0f, 1.0f, 1.0f, 1.0f);
}

/**
* Get launch charge level.
***********************************************************************************/

float UHUDWidgetComponent::GetLaunchChargeLevel() const
{
	return 0.0f;
}

/**
* Get the launch charge color.
***********************************************************************************/

FLinearColor UHUDWidgetComponent::GetLaunchChargeColour() const
{
	return FLinearColor(0.0f, 0.0f, 0.0f, 0.0f);
}

/**
* Get reset charge level.
***********************************************************************************/

float UHUDWidgetComponent::GetResetChargeLevel() const
{
	return 0.0f;
}

/**
* Get the reset charge color.
***********************************************************************************/

FLinearColor UHUDWidgetComponent::GetResetChargeColour() const
{
	return FLinearColor(0.0f, 0.0f, 0.0f, 0.0f);
}

/**
* Is a particular weapon of interest to the HUD?
***********************************************************************************/

bool UHUDWidgetComponent::IsWeaponEventOfInterest(int32 index) const
{
	TArray<FGameEvent>& events = PlayGameMode->GameEvents;

	if (index < events.Num())
	{
		FGameEvent& event = events[index];

		if (event.EventType == EGameEventType::Blocked ||
			event.EventType == EGameEventType::Destroyed ||
			event.EventType == EGameEventType::ChatMessage)
		{
			return true;
		}

		if (event.EventType == EGameEventType::Impacted &&
			(event.PickupUsed == EPickupType::HomingMissile))
		{
			return true;
		}
	}

	return false;
}

/**
* Get the left-hand text for a weapon event.
***********************************************************************************/

FText UHUDWidgetComponent::GetWeaponEventLeftText(int32 index) const
{
	TArray<FGameEvent>& events = PlayGameMode->GameEvents;

	if (index < events.Num())
	{
		FGameEvent& event = events[index];

		if (event.LaunchVehicleIndex >= 0)
		{
			return FText::FromString(PlayGameMode->GetVehicleForVehicleIndex(event.LaunchVehicleIndex)->GetPlayerName(true, true));
		}
	}

	return FText::FromString("");
}

/**
* Get the right-hand text for a weapon event.
***********************************************************************************/

FText UHUDWidgetComponent::GetWeaponEventRightText(int32 index) const
{
	TArray<FGameEvent>& events = PlayGameMode->GameEvents;

	if (index < events.Num())
	{
		FGameEvent& event = events[index];

		if (event.EventType == EGameEventType::ChatMessage)
		{
			return FText::FromString(" " + event.ExtraInformation);
		}
		else
		{
			if (event.TargetVehicleIndex >= 0)
			{
				return FText::FromString(PlayGameMode->GetVehicleForVehicleIndex(event.TargetVehicleIndex)->GetPlayerName(true, true));
			}
		}
	}

	return FText::FromString("");
}

/**
* Get the left-hand color for a weapon event.
***********************************************************************************/

FSlateColor UHUDWidgetComponent::GetWeaponEventLeftColour(int32 index) const
{
	TArray<FGameEvent>& events = PlayGameMode->GameEvents;

	if (index < events.Num())
	{
		FGameEvent& event = events[index];

		if (event.EventType == EGameEventType::ChatMessage)
		{
			return FSlateColor(FLinearColor(1.0f, 0.5f, 0.0f, 1.0f));
		}
		else if (event.EventType == EGameEventType::Impacted)
		{
			if (OwningVehicle != nullptr)
			{
				if (event.LaunchVehicleIndex == OwningVehicle->VehicleIndex)
				{
					return FSlateColor(FLinearColor(0.0f, 1.0f, 0.0f, 1.0f));
				}
			}
		}
	}

	return FSlateColor(FLinearColor(1.0f, 1.0f, 1.0f, 1.0f));
}

/**
* Get the right-hand color for a weapon event.
***********************************************************************************/

FSlateColor UHUDWidgetComponent::GetWeaponEventRightColour(int32 index) const
{
	TArray<FGameEvent>& events = PlayGameMode->GameEvents;

	if (index < events.Num())
	{
		FGameEvent& event = events[index];
		ABaseVehicle* vehicle = GetTargetVehicle();

		if (event.EventType == EGameEventType::ChatMessage)
		{
			return FSlateColor(FLinearColor(1.0f, 1.0f, 1.0f, 1.0f));
		}
		else if ((event.EventType == EGameEventType::Impacted && vehicle != nullptr && event.TargetVehicleIndex == vehicle->VehicleIndex) ||
			event.EventType == EGameEventType::Destroyed)
		{
			return FSlateColor(FLinearColor(1.0f, 0.0f, 0.0f, 1.0f));
		}
	}

	return FSlateColor(FLinearColor(1.0f, 1.0f, 1.0f, 1.0f));
}

/**
* Get the image index for a weapon event.
***********************************************************************************/

int32 UHUDWidgetComponent::GetWeaponEventImageIndex(int32 index) const
{
	TArray<FGameEvent>& events = PlayGameMode->GameEvents;

	if (index < events.Num())
	{
		FGameEvent& event = events[index];

		if (event.EventType == EGameEventType::Impacted)
		{
			if (event.PickupUsed == EPickupType::HomingMissile)
			{
				return 0;
			}
		}
		else if (event.EventType == EGameEventType::Blocked)
		{
			return 3;
		}
		else if (event.EventType == EGameEventType::Destroyed)
		{
			return 4;
		}
	}

	return -1;
}

/**
* Get the text describing a race distance.
***********************************************************************************/

FText UHUDWidgetComponent::GetRacePositionScaleText(int32 distance) const
{
	FFormatNamedArguments arguments;

	arguments.Add(TEXT("Distance"), FText::AsNumber(distance));

	return FText::Format(NSLOCTEXT("GripHUD", "RaceDistance", "{Distance} m"), arguments);
}

/**
* Get the scale for rendering the race positions for the players.
***********************************************************************************/

int32 UHUDWidgetComponent::GetRacePositionScale() const
{
	if (GetTargetVehicle() != nullptr)
	{
		float maxDistance = 0.0f;
		ABaseVehicle* yourVehicle = GetTargetVehicle();

		if (yourVehicle != nullptr)
		{
			int32 you = GetTargetVehicle()->VehicleIndex;
			TArray<ABaseVehicle*>& vehicles = PlayGameMode->GetVehicles();

			for (ABaseVehicle* vehicle : vehicles)
			{
				if (vehicle->VehicleIndex != you &&
					vehicle->IsVehicleDestroyed() == false)
				{
					maxDistance = FMath::Max(maxDistance, FMath::Abs(vehicle->GetRaceState().RaceDistance - yourVehicle->GetRaceState().RaceDistance));
				}
			}

			float packLength = FMath::Clamp(maxDistance, 500.0f * 100.0f, 2000.0f * 100.0f);

			return (int32)(packLength / 100.0f);
		}
	}

	return 0;
}

/**
* Get all of the race positions for all of the players.
***********************************************************************************/

TArray<FHUDRacePosition> UHUDWidgetComponent::GetRacePositions(float centre, float length) const
{
	TArray<FHUDRacePosition> positions;

	ABaseVehicle* thisVehicle = GetTargetVehicle();

	if (thisVehicle != nullptr)
	{
		float packLength = (float)GetRacePositionScale();
		TArray<ABaseVehicle*>& vehicles = PlayGameMode->GetVehicles();

		positions.Reserve(vehicles.Num() - 1);

		for (ABaseVehicle* vehicle : vehicles)
		{
			if (vehicle->IsVehicleDestroyed() == false &&
				vehicle != thisVehicle)
			{
				FHUDRacePosition position;

				float relativeWorldDistance = (vehicle->GetRaceState().RaceDistance - thisVehicle->GetRaceState().RaceDistance) / 100.0f;

				position.RelativeWidgetDistance = FMath::Clamp(relativeWorldDistance / packLength, -1.0f, 1.0f);
				position.RelativeWidgetDistance = (position.RelativeWidgetDistance * length * -0.5f) + centre;
				position.IsRival = false;
				position.IsUsingDisruptor = false;
				position.Colour = PlayGameMode->GetTeamColour(-1);

				float teleportRatio = 0.0f;

				position.Colour = FMath::Lerp(FLinearColor(1.0f, 0.33f, 0.0f, 1.0f), position.Colour, teleportRatio * teleportRatio * teleportRatio);

				positions.Emplace(position);
			}
		}
	}

	return positions;
}

/**
* Record all of the widgets that can be ignited within a particular panel.
***********************************************************************************/

void UHUDWidgetComponent::AddIgnitionWidgets(UPanelWidget* mainPanel)
{
	for (int32 i = 0; i < mainPanel->GetChildrenCount(); i++)
	{
		UPanelWidget* panel = Cast<UPanelWidget>(mainPanel->GetChildAt(i));

		if (panel != nullptr)
		{
			AddIgnitionWidgets(panel);
		}
		else
		{
			UUserWidget* userWidget = Cast<UUserWidget>(mainPanel->GetChildAt(i));

			if (userWidget != nullptr)
			{
				IgnitionWidgets.Emplace(FHUDPanelIgnition(userWidget, FMath::FRandRange(3.0f, 4.0f)));
			}
		}
	}
}

/**
* Refresh all of the widgets that can be ignited.
***********************************************************************************/

void UHUDWidgetComponent::RefreshIgnitionWidgets()
{
	for (FHUDPanelIgnition& widget : IgnitionWidgets)
	{
		if (UCanvasPanel* panel = Cast<UCanvasPanel>(widget.Widget->GetRootWidget()))
		{
			RefreshIgnitionWidgets(widget, panel);
		}
	}
}

/**
* Refresh all of the widgets that can be ignited for a particular panel.
***********************************************************************************/

void UHUDWidgetComponent::RefreshIgnitionWidgets(FHUDPanelIgnition& ignition, UPanelWidget* mainPanel)
{
	float opacity = GetPanelOpacity(ignition);

	for (int32 i = 0; i < mainPanel->GetChildrenCount(); i++)
	{
		UWidget* child = mainPanel->GetChildAt(i);
		UBorder* border = Cast<UBorder>(child);

		if (border != nullptr)
		{
			FLinearColor color = border->ContentColorAndOpacity;

			color.A = opacity;

			border->SetBrushColor(color);
			border->SetContentColorAndOpacity(color);
		}

		UPanelWidget* panel = Cast<UPanelWidget>(child);

		if (panel != nullptr)
		{
			RefreshIgnitionWidgets(ignition, panel);
		}
		else
		{
			UTextBlock* textBlock = Cast<UTextBlock>(child);

			if (textBlock != nullptr)
			{
				textBlock->SetOpacity(opacity);
			}
		}
	}
}

/**
* Get the angle of the ground from the perspective of the camera.
***********************************************************************************/

float UHUDWidgetComponent::GetCameraGroundAngle() const
{
	if (OwningVehicle != nullptr)
	{
		FMinimalViewInfo view;

		OwningVehicle->Camera->GetCameraViewNoPostProcessing(0.0f, view);

		return 180.0f - view.Rotation.Roll;
	}

	return 0.0f;
}

/**
* Draw a vehicle diagnostics chart.
***********************************************************************************/

void UHUDWidgetComponent::DrawVehicleDiagnosticsChart(UPARAM(ref) FPaintContext& context, USlateBrushAsset* brush, const ABaseVehicle* vehicle, float x, float y, float width, float height, const FTimedFloatList& list, float scale, bool angles, bool calculateDifference, const FString& title)
{
	UWidgetBlueprintLibrary::DrawBox(context, FVector2D(x, y), FVector2D(width, height), brush, FLinearColor(1.0f, 1.0f, 1.0f, 1.0f));

	UWidgetBlueprintLibrary::DrawText(context, title, FVector2D(x + 10, y + 10), FLinearColor(0.0f, 0.0f, 0.0f, 1.0f));

	UWidgetBlueprintLibrary::DrawLine(context, FVector2D(x, y + (height * 0.5f)), FVector2D(x + width, y + (height * 0.5f)), FLinearColor(1.0f, 0.0f, 0.0f, 1.0f), false);

	TArray<FVector2D> border;

	border.Emplace(FVector2D(x, y));
	border.Emplace(FVector2D(x + width, y));
	border.Emplace(FVector2D(x + width, y + height));
	border.Emplace(FVector2D(x, y + height));
	border.Emplace(FVector2D(x, y));

	UWidgetBlueprintLibrary::DrawLines(context, border, FLinearColor(0.0f, 0.0f, 0.0f, 1.0f));

	TArray<FVector2D> yawChart;

	float numLines = list.GetNumValues() - 1.0f;
	float lineWidth = width / numLines;
	float last = list[0].Value;

	for (int32 i = 1; i < list.GetNumValues(); i++)
	{
		FVector2D line;
		float value = list[i].Value;
		float difference = value;

		if (calculateDifference == true)
		{
			difference = value - last;

			if (angles == true)
			{
				if (difference < -180.0f)
				{
					difference = 360.0f + difference;
				}
				else if (difference > 180.0f)
				{
					difference = -360.0f + difference;
				}
			}
		}

		line.X = (i - 1) * lineWidth;
		line.Y = height * 0.5f + (difference * scale);
		line.Y = FMath::Clamp(line.Y, 0.0f, height);

		yawChart.Emplace(FVector2D(x, y) + line);

		last = list[i].Value;
	}

	UWidgetBlueprintLibrary::DrawLines(context, yawChart, FLinearColor(0.0f, 0.0f, 0.0f, 1.0f), false);
}

/**
* Get the progress through the event between 0 and 1.
***********************************************************************************/

float UHUDWidgetComponent::GetEventProgress() const
{ return PlayGameMode->GetEventProgress(); }

/**
* Draw the vehicle diagnostics charts.
***********************************************************************************/

void UHUDWidgetComponent::DrawVehicleDiagnostics(UPARAM(ref) FPaintContext& context, USlateBrushAsset* brush, const ABaseVehicle* vehicle)
{
#if 0
	float width = 400.0f;
	float height = 125.0f;
	float border = 25.0f;
	float viewWidth = context.MyClippingRect.Right - context.MyClippingRect.Left;
	float viewHeight = context.MyClippingRect.Bottom - context.MyClippingRect.Top;
	float x = viewWidth - (width + 40.0f);
	float y = (viewHeight * 0.5f) - (height + border + height * 0.5f);

	const FVehiclePhysics& physics = vehicle->GetPhysics();

	if (physics.SteeringPositionList.IsFull() == true)
	{
		DrawVehicleDiagnosticsChart(context, brush, vehicle, x, y, width, height, physics.SteeringPositionList, height * 0.45f, true, false, "Steering position");

		y += height + border;
	}

	if (physics.YawChangeList.IsFull() == true)
	{
		DrawVehicleDiagnosticsChart(context, brush, vehicle, x, y, width, height, physics.YawChangeList, 75.0f, true, false, "Local direction yaw change");

		y += height + border;
	}

	if (physics.VelocityYawList.IsFull() == true)
	{
		DrawVehicleDiagnosticsChart(context, brush, vehicle, x, y, width, height, physics.VelocityYawList, 4.0f, true, false, "Local velocity yaw");

		y += height + border;
	}
#endif
}

/**
* Construct a UHUDWidget.
***********************************************************************************/

UHUDWidget::UHUDWidget(const FObjectInitializer& objectInitializer)
	: Super(objectInitializer)
{ }

/**
* Setup the HUD widget for a particular player.
***********************************************************************************/

void UHUDWidget::SetupForPlayer(ULocalPlayer* localPlayer)
{
	if (localPlayer == nullptr)
	{
		APlayerController* controller = UGameplayStatics::GetPlayerController(this, 0);

		if (controller != nullptr)
		{
			localPlayer = controller->GetLocalPlayer();
		}
	}

	SetOwningLocalPlayer(localPlayer);

	SetupForGameMode(localPlayer);
}

/**
* Setup the components of the HUD widget.
***********************************************************************************/

void UHUDWidget::SetupComponents(UCanvasPanel* mainPanel) const
{
	TArray<int32> removeIndices;
	EDrivingMode drivingMode = GetDrivingMode();
	ULocalPlayer* owningPlayer = GetOwningPlayer()->GetLocalPlayer();

	for (int32 i = 0; i < mainPanel->GetChildrenCount(); i++)
	{
		UHUDWidgetComponent* component = Cast<UHUDWidgetComponent>(mainPanel->GetChildAt(i));

		if (component != nullptr)
		{
			UHUDWidgetComponent* hudComponent = Cast<UHUDWidgetComponent>(component);
			bool valid = true;

			valid &= (hudComponent->OnlineOnly == false);
			valid &= (hudComponent->SpectatorOnly == false);
			valid &= (hudComponent->RestrictDrivingModes == false || hudComponent->DrivingModes.Contains(drivingMode) == true);

			if (hudComponent != nullptr &&
				valid == false)
			{
				removeIndices.Emplace(i);
				continue;
			}
			else if (hudComponent != nullptr &&
				hudComponent->UseIgnition == true)
			{
				UWidget* root = hudComponent->GetRootWidget();

				if (UCanvasPanel* panel = Cast<UCanvasPanel>(root))
				{
					hudComponent->AddIgnitionWidgets(panel);
					hudComponent->RefreshIgnitionWidgets();
				}
			}

			component->SetOwningLocalPlayer(owningPlayer);
			component->SetupHUDWidget(drivingMode);
		}
	}

	int32 numRemoved = 0;

	for (int32 index : removeIndices)
	{
		UWidget* child = mainPanel->GetChildAt(index - numRemoved);

		child->SetVisibility(ESlateVisibility::Collapsed);

		mainPanel->RemoveChildAt(index - numRemoved++);
	}
}

/**
* Update the HUD widget.
***********************************************************************************/

void UHUDWidget::Update(float deltaSeconds)
{
	if (PlayGameMode->GetClock() > 0.0f &&
		PlayGameMode->GetClock() - PlayGameMode->LastOptionsResetTime < 10.0f)
	{
		UWidget* root = GetRootWidget();

		if (UCanvasPanel* panel = Cast<UCanvasPanel>(root))
		{
			for (int32 i = 0; i < panel->GetChildrenCount(); i++)
			{
				UHUDWidgetComponent* hudComponent = Cast<UHUDWidgetComponent>(panel->GetChildAt(i));

				if (hudComponent != nullptr &&
					hudComponent->UseIgnition == true)
				{
					hudComponent->UpdateIgnition(deltaSeconds);
				}
			}
		}
	}
}

/**
* Ignite the HUD widget and have the elements flicker on over time.
***********************************************************************************/

void UHUDWidget::Ignite()
{
	if (Ignited == false)
	{
		Ignited = true;

		PlayGameMode->LastOptionsResetTime = PlayGameMode->GetClock();

		UWidget* root = GetRootWidget();

		if (UCanvasPanel* panel = Cast<UCanvasPanel>(root))
		{
			for (int32 i = 0; i < panel->GetChildrenCount(); i++)
			{
				UHUDWidgetComponent* hudComponent = Cast<UHUDWidgetComponent>(panel->GetChildAt(i));

				if (hudComponent != nullptr &&
					hudComponent->UseIgnition == true)
				{
					hudComponent->Ignite();
				}
			}
		}
	}
}

/**
* Tick the HUD widget.
***********************************************************************************/

void UHUDWidget::NativeTick(const FGeometry& MyGeometry, float InDeltaTime)
{
	Super::NativeTick(MyGeometry, InDeltaTime);

	ABaseVehicle* vehicle = GetOwningVehicle();

	if (vehicle != nullptr &&
		vehicle->GetRaceState().LapNumber != LastLapCompleted &&
		vehicle->GetRaceState().LapNumber > 0)
	{
		LapCompleted(vehicle->GetRaceState().PlayerCompletionState < EPlayerCompletionState::Complete &&
			GameState->IsGameModeLapBased() == true &&
			vehicle->GetRaceState().LapNumber == GameState->GeneralOptions.NumberOfLaps - 1);

		LastLapCompleted = vehicle->GetRaceState().LapNumber;
	}
}
