/**
*
* Vehicle mesh implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* A vehicle mesh component, derived from USkeletalMeshComponent, which contains a
* lot of functionality for physics and sub-stepping.
*
* Define GRIP_ENGINE_PHYSICS_MODIFIED as 1 if the physics system in the engine has
* been modified to deal with sub-stepping correctly, or to 0 if using a stock
* Unreal Engine version.
*
***********************************************************************************/

#include "Vehicle/VehicleMeshComponent.h"

#pragma region Vehicle

/**
* Do some initialization when the game is ready to play.
***********************************************************************************/

void UVehicleMeshComponent::BeginPlay()
{
	Super::BeginPlay();

	if (GetBodyInstance() != nullptr)
	{
		ActorHandle = GetBodyInstance()->GetPhysicsActorHandle();
	}
}

/**
* Get the physics location of the vehicle.
***********************************************************************************/

FVector UVehicleMeshComponent::GetPhysicsLocation() const
{
	FVector result = FVector::ZeroVector;

	FPhysicsCommand::ExecuteRead(ActorHandle, [&] (const FPhysicsActorHandle& actor)
		{
			const FTransform& transform = FPhysicsInterface::GetGlobalPose_AssumesLocked(actor);

			result = transform.GetTranslation();
		});

	return result;
}

/**
* Get the physics rotator of the vehicle.
***********************************************************************************/

FRotator UVehicleMeshComponent::GetPhysicsRotator() const
{
	FRotator result = FRotator::ZeroRotator;

	FPhysicsCommand::ExecuteRead(ActorHandle, [&] (const FPhysicsActorHandle& actor)
		{
			const FTransform& transform = FPhysicsInterface::GetGlobalPose_AssumesLocked(actor);

			result = transform.GetRotation().Rotator();
		});

	return result;
}

/**
* Get the physics quaternion of the vehicle.
***********************************************************************************/

FQuat UVehicleMeshComponent::GetPhysicsQuaternion() const
{
	FQuat result = FQuat::Identity;

	FPhysicsCommand::ExecuteRead(ActorHandle, [&] (const FPhysicsActorHandle& actor)
		{
			const FTransform& transform = FPhysicsInterface::GetGlobalPose_AssumesLocked(actor);

			result = transform.GetRotation();
		});

	return result;
}

/**
* Get the physics transform of the vehicle.
***********************************************************************************/

FTransform UVehicleMeshComponent::GetPhysicsTransform() const
{
	FTransform result = FTransform::Identity;

	FPhysicsCommand::ExecuteRead(ActorHandle, [&] (const FPhysicsActorHandle& actor)
		{
			result = FPhysicsInterface::GetGlobalPose_AssumesLocked(actor);
		});

	return result;
}

/**
* Get the physics linear velocity of the vehicle.
***********************************************************************************/

FVector UVehicleMeshComponent::GetPhysicsLinearVelocity(const FName& boneName) const
{
	check(boneName == NAME_None);

	FVector result = FVector::ZeroVector;

	FPhysicsCommand::ExecuteRead(ActorHandle, [&] (const FPhysicsActorHandle& actor)
		{
			result = FPhysicsInterface::GetLinearVelocity_AssumesLocked(actor);
		});

	return result;
}

/**
* Get the physics linear velocity of a point relative the vehicle in world space.
***********************************************************************************/

FVector UVehicleMeshComponent::GetPhysicsLinearVelocityAtPoint(const FVector& point, const FName& boneName) const
{
	check(boneName == NAME_None);

	FVector result = FVector::ZeroVector;

	FPhysicsCommand::ExecuteRead(ActorHandle, [&] (const FPhysicsActorHandle& actor)
		{
			result = FPhysicsInterface::GetWorldVelocityAtPoint_AssumesLocked(actor, point);
		});

	return result;
}

/**
* Get the physics angular velocity of the vehicle.
***********************************************************************************/

FVector UVehicleMeshComponent::GetPhysicsAngularVelocityInRadians(const FName& boneName) const
{
	check(boneName == NAME_None);

	FVector result = FVector::ZeroVector;

	FPhysicsCommand::ExecuteRead(ActorHandle, [&] (const FPhysicsActorHandle& actor)
		{
			result = FPhysicsInterface::GetAngularVelocity_AssumesLocked(actor);
		});

	return result;
}

/**
* Set the physics inertia tensor of the vehicle.
***********************************************************************************/

FVector UVehicleMeshComponent::GetPhysicsInertiaTensor() const
{
	FVector result = FVector::ZeroVector;

	FPhysicsCommand::ExecuteRead(ActorHandle, [&] (const FPhysicsActorHandle& actor)
		{
			result = FPhysicsInterface::GetLocalInertiaTensor_AssumesLocked(actor);
		});

	return result;
}

/**
* Set the physics location and quaternion of the vehicle.
***********************************************************************************/

void UVehicleMeshComponent::SetPhysicsLocationAndQuaternionSubstep(const FVector& location, const FQuat& rotation)
{
	check(location.ContainsNaN() == false);
	check(rotation.ContainsNaN() == false);

	FPhysicsCommand::ExecuteWrite(ActorHandle, [&] (const FPhysicsActorHandle& actor)
		{
			FTransform transform = FPhysicsInterface::GetGlobalPose_AssumesLocked(ActorHandle);

			transform.SetTranslation(location);
			transform.SetRotation(rotation);

			FPhysicsInterface::SetGlobalPose_AssumesLocked(actor, transform);
		});
}

/**
* Set the physics mass and inertia tensor of the vehicle.
***********************************************************************************/

void UVehicleMeshComponent::SetPhysicsMassAndInertiaTensorSubstep(float mass, const FVector& inertiaTensor)
{
	check(FMath::IsFinite(mass) == true);
	check(inertiaTensor.ContainsNaN() == false);

	FPhysicsCommand::ExecuteWrite(ActorHandle, [&] (const FPhysicsActorHandle& actor)
		{
			FPhysicsInterface::SetMass_AssumesLocked(actor, mass);
			FPhysicsInterface::SetMassSpaceInertiaTensor_AssumesLocked(actor, inertiaTensor * mass);
		});
}

/**
* Set the physics linear velocity of the vehicle.
***********************************************************************************/

void UVehicleMeshComponent::SetPhysicsLinearVelocitySubstep(FVector velocity, bool addToCurrent, const FName& boneName)
{
	check(velocity.ContainsNaN() == false);
	check(boneName == NAME_None);

#if GRIP_ENGINE_PHYSICS_MODIFIED
	FPhysicsCommand::ExecuteWrite(ActorHandle, [&] (const FPhysicsActorHandle& actor)
		{
			if (addToCurrent == true)
			{
				velocity += FPhysicsInterface::GetLinearVelocity_AssumesLocked(actor);
			}

			FPhysicsInterface::SetLinearVelocity_AssumesLocked(actor, velocity);
		});
#else // GRIP_ENGINE_PHYSICS_MODIFIED
	SetPhysicsLinearVelocity(velocity, addToCurrent, boneName);
#endif // GRIP_ENGINE_PHYSICS_MODIFIED
}

/**
* Set the physics angular velocity of the vehicle.
***********************************************************************************/

void UVehicleMeshComponent::SetPhysicsAngularVelocityInRadiansSubstep(FVector angularVelocity, bool addToCurrent, const FName& boneName)
{
	check(angularVelocity.ContainsNaN() == false);
	check(boneName == NAME_None);

#if GRIP_ENGINE_PHYSICS_MODIFIED
	FPhysicsCommand::ExecuteWrite(ActorHandle, [&] (const FPhysicsActorHandle& actor)
		{
			if (addToCurrent == true)
			{
				angularVelocity += FPhysicsInterface::GetAngularVelocity_AssumesLocked(actor);
			}

			FPhysicsInterface::SetAngularVelocity_AssumesLocked(actor, angularVelocity);
		});
#else // GRIP_ENGINE_PHYSICS_MODIFIED
	SetPhysicsAngularVelocityInRadians(angularVelocity, addToCurrent, boneName);
#endif // GRIP_ENGINE_PHYSICS_MODIFIED
}

/**
* Add a force to the vehicle.
***********************************************************************************/

void UVehicleMeshComponent::AddForceSubstep(FVector force, const FName& boneName, bool accelerationChange)
{
	check(force.ContainsNaN() == false);
	check(boneName == NAME_None);

#if GRIP_ENGINE_PHYSICS_MODIFIED
	if (IsIdleLocked() == false)
	{
		FPhysicsCommand::ExecuteWrite(ActorHandle, [&] (const FPhysicsActorHandle& actor)
			{
				if (accelerationChange == true)
				{
					// Not strictly correct, but correct enough.

					force *= FPhysicsInterface::GetMass_AssumesLocked(actor);
				}

				FPhysicsInterface::AddForce_AssumesLocked(actor, force);
			});
	}
#else // GRIP_ENGINE_PHYSICS_MODIFIED
	AddForce(force, boneName, accelerationChange);
#endif // GRIP_ENGINE_PHYSICS_MODIFIED
}

/**
* Add a force to the vehicle at a particular location in world space.
***********************************************************************************/

void UVehicleMeshComponent::AddForceAtLocationSubstep(const FVector& force, const FVector& location, const FName& boneName)
{
	check(force.ContainsNaN() == false);
	check(location.ContainsNaN() == false);
	check(boneName == NAME_None);

#if GRIP_ENGINE_PHYSICS_MODIFIED
	if (IsIdleLocked() == false)
	{
		FPhysicsCommand::ExecuteWrite(ActorHandle, [&] (const FPhysicsActorHandle& actor)
			{
				FPhysicsInterface::AddForceAtLocation_AssumesLocked(actor, force, location);
			});
	}
#else // GRIP_ENGINE_PHYSICS_MODIFIED
	AddForceAtLocation(force, location, boneName);
#endif // GRIP_ENGINE_PHYSICS_MODIFIED
}

/**
* Add a force to the vehicle at a particular location in local space.
***********************************************************************************/

void UVehicleMeshComponent::AddForceAtLocationLocalSubstep(const FVector& force, const FVector& location, const FName& boneName)
{
	check(force.ContainsNaN() == false);
	check(location.ContainsNaN() == false);

#if GRIP_ENGINE_PHYSICS_MODIFIED
	AddForceAtLocation(force, GetPhysicsTransform().TransformPosition(location), boneName);
#else // GRIP_ENGINE_PHYSICS_MODIFIED
	AddForceAtLocationLocal(force, location, boneName);
#endif // GRIP_ENGINE_PHYSICS_MODIFIED
}

/**
* Add a force to the vehicle, originating from the origin in world space.
***********************************************************************************/

void UVehicleMeshComponent::AddRadialForceSubstep(const FVector& origin, float radius, float strength, enum ERadialImpulseFalloff falloff, bool accelerationChange)
{
	// We have no implementation for this.

	check(0);
}

/**
* Add an impulse to the vehicle. Good for one time instant burst.
***********************************************************************************/

void UVehicleMeshComponent::AddImpulseSubstep(const FVector& impulse, const FName& boneName, bool velocityChange)
{
	check(impulse.ContainsNaN() == false);
	check(boneName == NAME_None && velocityChange == false);

#if GRIP_ENGINE_PHYSICS_MODIFIED
	if (IsIdleLocked() == false)
	{
		FPhysicsCommand::ExecuteWrite(ActorHandle, [&] (const FPhysicsActorHandle& actor)
			{
				if (velocityChange == true)
				{
					FPhysicsInterface::AddVelocity_AssumesLocked(actor, impulse);
				}
				else
				{
					FPhysicsInterface::AddImpulse_AssumesLocked(actor, impulse);
				}
			});
	}
#else // GRIP_ENGINE_PHYSICS_MODIFIED
	AddImpulse(impulse, boneName, velocityChange);
#endif // GRIP_ENGINE_PHYSICS_MODIFIED
}

/**
* Add an impulse to the vehicle at a specific location.
***********************************************************************************/

void UVehicleMeshComponent::AddImpulseAtLocationSubstep(const FVector& impulse, const FVector& location, const FName& boneName)
{
	check(impulse.ContainsNaN() == false);
	check(location.ContainsNaN() == false);
	check(boneName == NAME_None);

#if GRIP_ENGINE_PHYSICS_MODIFIED
	if (IsIdleLocked() == false)
	{
		FPhysicsCommand::ExecuteWrite(ActorHandle, [&] (const FPhysicsActorHandle& actor)
			{
				FPhysicsInterface::AddImpulseAtLocation_AssumesLocked(actor, impulse, location);
			});
	}
#else // GRIP_ENGINE_PHYSICS_MODIFIED
	AddImpulseAtLocation(impulse, location, boneName);
#endif // GRIP_ENGINE_PHYSICS_MODIFIED
}

/**
* Add an impulse to the vehicle, radiating out from the origin in world space.
***********************************************************************************/

void UVehicleMeshComponent::AddRadialImpulseSubstep(const FVector& origin, float radius, float strength, enum ERadialImpulseFalloff falloff, bool velocityChange)
{
	check(origin.ContainsNaN() == false);
	check(FMath::IsFinite(radius) == true);
	check(FMath::IsFinite(strength) == true);

#if GRIP_ENGINE_PHYSICS_MODIFIED
	if (IsIdleLocked() == false)
	{
		FPhysicsCommand::ExecuteWrite(ActorHandle, [&] (const FPhysicsActorHandle& actor)
			{
				FPhysicsInterface::AddRadialImpulse_AssumesLocked(actor, origin, radius, strength, falloff, velocityChange);
			});
	}
#else // GRIP_ENGINE_PHYSICS_MODIFIED
	AddRadialImpulse(origin, radius, strength, falloff, velocityChange);
#endif // GRIP_ENGINE_PHYSICS_MODIFIED
}

/**
* Add a rotational impulse to this body in radians.
***********************************************************************************/

void UVehicleMeshComponent::AddAngularImpulseInRadiansSubstep(FVector angularImpulse, FName boneName, bool velocityChange)
{
	check(angularImpulse.ContainsNaN() == false);
	check(boneName == NAME_None);

#if GRIP_ENGINE_PHYSICS_MODIFIED
	if (IsIdleLocked() == false)
	{
		FPhysicsCommand::ExecuteWrite(ActorHandle, [&] (const FPhysicsActorHandle& actor)
			{
				if (velocityChange == true)
				{
					FPhysicsInterface::AddAngularVelocityInRadians_AssumesLocked(actor, angularImpulse);
				}
				else
				{
					FPhysicsInterface::AddAngularImpulseInRadians_AssumesLocked(actor, angularImpulse);
				}
			});
	}
#else // GRIP_ENGINE_PHYSICS_MODIFIED
	AddAngularImpulseInRadians(angularImpulse, boneName, velocityChange);
#endif // GRIP_ENGINE_PHYSICS_MODIFIED
}

#pragma endregion Vehicle
