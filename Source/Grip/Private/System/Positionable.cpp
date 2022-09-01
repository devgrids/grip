/**
*
* Positionable interface.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* An interface to use for having objects clamped to the nearest surface in the scene.
*
***********************************************************************************/

#include "system/positionable.h"
#include "gamemodes/basegamemode.h"

/**
* Construct a UPositionableInterface.
***********************************************************************************/

UPositionableInterface::UPositionableInterface(const FObjectInitializer& objectInitializer)
	: Super(objectInitializer)
{
}

/**
* Determine the surface position / rotation for an actor.
***********************************************************************************/

bool IPositionableInterface::DetermineSurfacePosition(FVector& position, FRotator& rotation, float radius, AActor* parent, ECollisionChannel channel, bool useRotation)
{
	FHitResult minHit;
	float minDistance = BIG_NUMBER;
	const int32 iterations = 8;

	if (channel == ECC_MAX)
	{
		channel = ABaseGameMode::ECC_LineOfSightTest;
	}

	if (useRotation == true)
	{
		FHitResult hit;
		FVector offset = rotation.Vector();
		FVector start = position;
		FVector end = position + (offset * radius);

		if (parent->GetWorld()->LineTraceSingleByChannel(hit, start, end, channel, FCollisionQueryParams(TEXT("Positionable"), true, parent)) == true)
		{
			FVector difference = (hit.ImpactPoint - position);

			minDistance = difference.SizeSquared();
			minHit = hit;
		}
	}
	else
	{
		for (int32 i = 0; i < iterations; i++)
		{
			for (int32 j = 0; j < iterations; j++)
			{
				FHitResult hit;
				FRotator thisRotation(((float)i / (float)iterations) * 360.0f, ((float)j / (float)iterations) * 360.0f, 0.0f);
				FVector offset = thisRotation.Vector();
				FVector start = position - (offset * radius);
				FVector end = position + (offset * radius);

				if (parent->GetWorld()->LineTraceSingleByChannel(hit, start, end, channel, FCollisionQueryParams(TEXT("Positionable"), true, parent)) == true)
				{
					FVector difference = (hit.ImpactPoint - position);
					float distance = difference.SizeSquared();

					if (minDistance > distance)
					{
						minDistance = distance;
						minHit = hit;
					}
				}
			}
		}
	}

	if (minDistance != BIG_NUMBER)
	{
		FQuat impactQuat = minHit.ImpactNormal.ToOrientationQuat();
		FQuat newRotation = impactQuat * FQuat(FRotator(-90.0f, 0.0f, 0.0f));

		if (FVector::DotProduct(newRotation.GetAxisZ(), rotation.Quaternion().GetAxisZ()) < 0.0f)
		{
			newRotation = impactQuat * FQuat(FRotator(90.0f, 0.0f, 0.0f));
		}

		rotation = newRotation.Rotator();
		position = minHit.ImpactPoint;

		return true;
	}

	return false;
}
