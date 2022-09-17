/**
*
* Vehicle debugging HUD.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
***********************************************************************************/

#include "ui/debugvehiclehud.h"
#include "vehicle/flippablevehicle.h"
#include "physicsengine/physicssettings.h"

#pragma region VehicleContactSensors

/**
* Draw the HUD.
***********************************************************************************/

void ADebugVehicleHUD::DrawHUD()
{
	Super::DrawHUD();

	HorizontalOffset = 200.0f;

	APawn* owningPawn = GetOwningPawn();
	ABaseVehicle* vehicle = Cast<ABaseVehicle>(owningPawn);

	if (vehicle != nullptr)
	{
		vehicle = vehicle->CameraTarget();
	}

	if (vehicle != nullptr)
	{
		AddBool(TEXT("IsFlipped"), vehicle->IsFlipped());
		AddBool(TEXT("IsFlippedAndWheelsOnGround"), vehicle->IsFlippedAndWheelsOnGround());
		AddBool(TEXT("IsPracticallyGrounded"), vehicle->IsPracticallyGrounded());
		AddFloat(TEXT("ContactData.ModeTime"), vehicle->Physics.ContactData.ModeTime);
		AddFloat(TEXT("GetSurfaceDistance"), FMath::Max(0.0f, vehicle->GetSurfaceDistance(false) - vehicle->GetMaxWheelRadius()));
		AddText(TEXT("GetSurfaceName"), FText::FromName(vehicle->GetSurfaceName()));
		AddFloat(TEXT("GetSpeedKPH"), vehicle->GetSpeedKPH());

#pragma region VehicleBasicForces

		AddInt(TEXT("GetJetEnginePower"), (int32)vehicle->GetJetEnginePower(vehicle->Wheels.NumWheelsInContact, vehicle->GetDirection()));
		AddInt(TEXT("GetDragForce"), (int32)vehicle->GetDragForce().Size());
		AddInt(TEXT("GetRollingResistance"), (int32)vehicle->GetRollingResistanceForce(vehicle->GetFacingDirection()).Size());
		AddFloat(TEXT("GetDownForce"), (vehicle->GetDownForce().Size() / vehicle->Physics.GravityStrength));
		AddFloat(TEXT("AutoBrakePosition"), vehicle->AutoBrakePosition(vehicle->GetFacingDirection()));

#pragma endregion VehicleBasicForces

		if (vehicle->Physics.Timing.TickCount > 0)
		{
			AddFloat(TEXT("General Clock"), vehicle->Physics.Timing.GeneralTickSum);
			AddFloat(TEXT("Physics Ticks Per Tick"), (float)vehicle->Physics.Timing.TickCount / (float)vehicle->Physics.Timing.GeneralTickCount);
			AddFloat(TEXT("Actual Tick Rate"), 1.0f / (vehicle->Physics.Timing.TickSum / (float)vehicle->Physics.Timing.TickCount));
			AddFloat(TEXT("Requested Tick Rate"), 1.0f / UPhysicsSettings::Get()->MaxSubstepDeltaTime);
		}

		AddBox(vehicle->GetActorLocation(), (vehicle->IsGrounded() == true) ? FLinearColor::Green : FLinearColor::Red);
		AddLine(vehicle->GetActorLocation(), vehicle->GetActorLocation() + vehicle->GetVelocityDirection() * 100.0f, (vehicle->IsGrounded() == true) ? FLinearColor::Green : FLinearColor::Red);

		// Show the suspension properties.

		if (vehicle->GetNumWheels() >= 4)
		{
			int32 index = 0;

			for (FVehicleWheel& wheel : vehicle->Wheels.Wheels)
			{
				const FTransform& transform = vehicle->GetPhysicsTransform();
				FVector wheelSpringPosition = vehicle->GetWheelBoneLocation(wheel, transform);
				FVector wheelPosition = vehicle->GetWheelBoneLocationFromIndex(index);

				float gripRatio = vehicle->GetGripRatio(wheel.GetActiveSensor());
				bool inContact = wheel.GetActiveSensor().IsInContact();
				float ratio = (inContact == true) ? FMath::Min(1.0f, gripRatio) : 0.0f;

				AddBox(vehicle->GetWheelBoneLocationFromIndex(index), FMath::Lerp(FLinearColor::Red, FLinearColor::Green, ratio));

#pragma region VehicleGrip

				FVector velocityDirection = vehicle->GetHorizontalVelocity(wheel);

				velocityDirection.Normalize();

				AddLine(vehicle->GetWheelBoneLocationFromIndex(index), wheelPosition + velocityDirection * 100.0f, FMath::Lerp(FLinearColor::Red, FLinearColor::Green, ratio));

#pragma endregion VehicleGrip

				FVector side = transform.TransformVector(FVector(0.0f, wheel.GetActiveSensor().GetSweepWidth(), 0.0f));

				AddLine(vehicle->GetWheelBoneLocationFromIndex(index), wheelPosition - side, FMath::Lerp(FLinearColor::Red, FLinearColor::Green, ratio));
				AddLine(vehicle->GetWheelBoneLocationFromIndex(index), wheelPosition + side, FMath::Lerp(FLinearColor::Red, FLinearColor::Green, ratio));

				{

#pragma region VehicleGrip

					AddTextFloatAt(TEXT("GR"), gripRatio, wheelPosition, -10.0f, -12.0f);

#pragma endregion VehicleGrip

					AddTextFloatAt(TEXT("CO"), wheel.GetActiveSensor().GetCompression(), wheelPosition, -10.0f, -24.0f);
					AddTextFloatAt(TEXT("NC"), wheel.GetActiveSensor().GetNormalizedCompression(), wheelPosition, -10.0f, -36.0f);

					float surfaceDistance = -transform.InverseTransformVector((wheel.GetActiveSensor().GetEndPoint() - wheelSpringPosition)).Z;

					if (vehicle->IsFlipped() == true)
					{
						surfaceDistance *= -1.0f;
					}

					AddTextIntAt(TEXT("SD"), (int32)(wheel.Radius - surfaceDistance), wheelPosition, -10.0f, 0.0f);

					for (FVehicleContactSensor& sensor : wheel.Sensors)
					{
						FVector springDirection = sensor.GetDirection();

						AddLine(vehicle->GetWheelBoneLocationFromIndex(index), vehicle->GetWheelBoneLocationFromIndex(index) + (springDirection * sensor.ForceApplied * 0.05f), FLinearColor(1.0f, 0.5f, 0.0f));

						if (sensor.ForceApplied != 0.0f)
						{
							AddTextIntAt(TEXT("FS"), (int32)(sensor.ForceApplied), vehicle->GetWheelBoneLocationFromIndex(index), -10.0f, -48.0f);
						}

						sensor.ForceApplied = 0.0f;
					}
				}

				index++;
			}
		}
	}
}

#pragma endregion VehicleContactSensors
