/**
*
* Debugging HUD for Metal in Motion.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
***********************************************************************************/

#include "UI/DebugHUD.h"
#include "Vehicle/BaseVehicle.h"
#include "AI/PursuitSplineComponent.h"
#include "UObject/ConstructorHelpers.h"

/**
* Construct the debugging HUD, mainly establishing a font to use for display.
***********************************************************************************/

ADebugHUD::ADebugHUD()
{
	static ConstructorHelpers::FObjectFinder<UFont> Font(TEXT("/Engine/EngineFonts/Roboto"));

	MainFont = Font.Object;
}

/**
* Add a route spline with a given distance and length to the HUD for rendering.
***********************************************************************************/

void ADebugHUD::AddRouteSpline(UPursuitSplineComponent* spline, float distance, float length, bool randomColor)
{
}
