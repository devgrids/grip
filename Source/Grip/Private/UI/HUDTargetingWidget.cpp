/**
*
* HUD targeting widgets implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Widget used specifically to draw symbology related to targeting onto the HUD.
*
***********************************************************************************/

#include "ui/hudtargetingwidget.h"
#include "blueprint/userwidget.h"
#include "blueprint/widgetblueprintlibrary.h"

/**
* Draw the primary homing symbology.
***********************************************************************************/

void UHUDTargetingWidgetComponent::DrawPrimaryHoming(const UHUDTargetingWidgetComponent* component, const FPaintContext& paintContext, USlateBrushAsset* slateBrush, float globalOpacity)
{
}

/**
* Draw the primary tracking symbology.
***********************************************************************************/

void UHUDTargetingWidgetComponent::DrawPrimaryTracking(const UHUDTargetingWidgetComponent* component, const FPaintContext& paintContext, USlateBrushAsset* slateBrush, USlateBrushAsset* slateBrushSecondary, float globalOpacity)
{
}

/**
* Draw the secondary tracking symbology.
***********************************************************************************/

void UHUDTargetingWidgetComponent::DrawSecondaryTracking(const UHUDTargetingWidgetComponent* component, const FPaintContext& paintContext, USlateBrushAsset* slateBrush, USlateBrushAsset* slateBrushSecondary, float globalOpacity)
{
}

/**
* Draw the threat symbology.
***********************************************************************************/

void UHUDTargetingWidgetComponent::DrawThreats(const UHUDTargetingWidgetComponent* component, const FPaintContext& paintContext, USlateBrushAsset* slateBrush, float globalOpacity)
{
}
