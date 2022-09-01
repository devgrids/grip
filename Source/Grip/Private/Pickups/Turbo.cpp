/**
*
* Turbo implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Turbo pickup type, one of the pickups used by vehicles in the game.
*
***********************************************************************************/

#include "pickups/turbo.h"
#include "vehicle/flippablevehicle.h"

/**
* Construct a turbo.
***********************************************************************************/

ATurbo::ATurbo()
{
	PickupType = EPickupType::TurboBoost;

	PrimaryActorTick.bCanEverTick = true;

	ActiveAudio = CreateDefaultSubobject<UAudioComponent>(TEXT("ActiveSound"));
	ActiveAudio->SetVolumeMultiplier(0.0f);

	SetRootComponent(ActiveAudio);

	BoostVsTime.GetRichCurve()->AddKey(0, 0.0f);
	BoostVsTime.GetRichCurve()->AddKey(1, 0.2f);
	BoostVsTime.GetRichCurve()->AddKey(2, 0.2f);
	BoostVsTime.GetRichCurve()->AddKey(3, 0.1f);
	BoostVsTime.GetRichCurve()->AddKey(4, 0.05f);
	BoostVsTime.GetRichCurve()->AddKey(5, 0.0f);
}
