/**
*
* Vehicle race camera implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* A specialist camera for racing vehicles, which contains a lot gizmos for
* enhancing the sensation of speed and works in conjunction with a specialist
* post processing material.
*
***********************************************************************************/

#include "camera/racecameracomponent.h"
#include "camera/flippablespringarmcomponent.h"
#include "game/globalgamestate.h"
#include "gamemodes/playgamemode.h"
#include "kismet/kismetmateriallibrary.h"
#include "vehicle/flippablevehicle.h"

/**
* Construct a race camera component.
***********************************************************************************/

URaceCameraComponent::URaceCameraComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;

	FieldOfViewVsSpeed.GetRichCurve()->AddKey(0.0f, 70.0f);
	FieldOfViewVsSpeed.GetRichCurve()->AddKey(120.0f, 82.0f);
	FieldOfViewVsSpeed.GetRichCurve()->AddKey(230.0f, 92.0f);
	FieldOfViewVsSpeed.GetRichCurve()->AddKey(350.0f, 97.0f);
	FieldOfViewVsSpeed.GetRichCurve()->AddKey(500.0f, 100.0f);

	RadialSpeedBlurVsSpeed.GetRichCurve()->AddKey(0.0f, 0.0f);
	RadialSpeedBlurVsSpeed.GetRichCurve()->AddKey(200.0f, 0.0f);
	RadialSpeedBlurVsSpeed.GetRichCurve()->AddKey(400.0f, 0.66f);
	RadialSpeedBlurVsSpeed.GetRichCurve()->AddKey(500.0f, 0.9f);
	RadialSpeedBlurVsSpeed.GetRichCurve()->AddKey(600.0f, 1.0f);

	IonisationVsSpeed.GetRichCurve()->AddKey(0.0f, 0.0f);
	IonisationVsSpeed.GetRichCurve()->AddKey(200.0f, 0.0f);
	IonisationVsSpeed.GetRichCurve()->AddKey(400.0f, 0.25f);
	IonisationVsSpeed.GetRichCurve()->AddKey(500.0f, 0.75f);
	IonisationVsSpeed.GetRichCurve()->AddKey(600.0f, 1.0f);

	StreaksVsSpeed.GetRichCurve()->AddKey(0.0f, 0.0f);
	StreaksVsSpeed.GetRichCurve()->AddKey(200.0f, 0.0f);
	StreaksVsSpeed.GetRichCurve()->AddKey(400.0f, 0.25f);
	StreaksVsSpeed.GetRichCurve()->AddKey(500.0f, 0.75f);
	StreaksVsSpeed.GetRichCurve()->AddKey(600.0f, 1.0f);
}
