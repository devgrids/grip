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

#pragma once

#include "system/gameconfiguration.h"
#include "ui/hudwidget.h"
#include "hudtargetingwidget.generated.h"

class USlateBrushAsset;

/**
* HUD targeting widget component.
***********************************************************************************/

UCLASS()
class GRIP_API UHUDTargetingWidgetComponent : public UHUDWidgetComponent
{
	GENERATED_BODY()

public:

	// Draw the primary homing symbology.
	UFUNCTION(BlueprintCallable, Category = HUD)
		static void DrawPrimaryHoming(const UHUDTargetingWidgetComponent* component, const FPaintContext& paintContext, USlateBrushAsset* slateBrush, float globalOpacity);

	// Draw the primary tracking symbology.
	UFUNCTION(BlueprintCallable, Category = HUD)
		static void DrawPrimaryTracking(const UHUDTargetingWidgetComponent* component, const FPaintContext& paintContext, USlateBrushAsset* slateBrush, USlateBrushAsset* slateBrushSecondary, float globalOpacity);

	// Draw the secondary tracking symbology.
	UFUNCTION(BlueprintCallable, Category = HUD)
		static void DrawSecondaryTracking(const UHUDTargetingWidgetComponent* component, const FPaintContext& paintContext, USlateBrushAsset* slateBrush, USlateBrushAsset* slateBrushSecondary, float globalOpacity);

	// Draw the threat symbology.
	UFUNCTION(BlueprintCallable, Category = HUD)
		static void DrawThreats(const UHUDTargetingWidgetComponent* component, const FPaintContext& paintContext, USlateBrushAsset* slateBrush, float globalOpacity);

private:

	// Get the size of a target from its opacity.
	FVector2D GetTargetSizeFromOpacity(float opacity, float size) const
	{ size = (opacity < 0.5f) ? FMath::Sin(opacity * PI) * size : size; return FVector2D(size, size); }
};
