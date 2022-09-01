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
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "CoreMinimal.h"
#include "Components/SkeletalMeshComponent.h"
#include "VehicleMeshComponent.generated.h"

#pragma region MinimalVehicle

/**
* UVehicleMeshComponent, derived from USkeletalMeshComponent, which contains a
* lot of functionality for physics and sub-stepping.
***********************************************************************************/

UCLASS(ClassGroup = Vehicle, meta = (BlueprintSpawnableComponent))
class GRIP_API UVehicleMeshComponent : public USkeletalMeshComponent
{
	GENERATED_BODY()

public:

	// Do some initialization when the game is ready to play.
	virtual void BeginPlay() override;

	// Get the physics location of the vehicle.
	FVector GetPhysicsLocation() const;

	// Get the physics rotator of the vehicle.
	FRotator GetPhysicsRotator() const;

	// Get the physics quaternion of the vehicle.
	FQuat GetPhysicsQuaternion() const;

	// Get the physics transform of the vehicle.
	FTransform GetPhysicsTransform() const;

	// Get the physics linear velocity of the vehicle.
	FVector GetPhysicsLinearVelocity(const FName& boneName = NAME_None) const;

	// Get the physics linear velocity of a point relative the vehicle in world space.
	FVector GetPhysicsLinearVelocityAtPoint(const FVector& point, const FName& boneName = NAME_None) const;

	// Get the physics angular velocity of the vehicle.
	FVector GetPhysicsAngularVelocityInRadians(const FName& boneName = NAME_None) const;

	// Get the physics angular velocity of the vehicle.
	FVector GetPhysicsAngularVelocityInDegrees(const FName& boneName = NAME_None) const
	{ return FMath::RadiansToDegrees(GetPhysicsAngularVelocityInRadians(boneName)); }

	// Set the physics inertia tensor of the vehicle.
	FVector GetPhysicsInertiaTensor() const;

	// Set the angular damping of the vehicle.
	virtual void SetAngularDamping(float damping) override
	{
		// We just pass this straight through as it's safe to do so.
		Super::SetAngularDamping(damping);
	}

	// Set the physics linear velocity of the vehicle.
	virtual void SetPhysicsLinearVelocity(FVector velocity, bool addToCurrent = false, FName boneName = NAME_None) override
	{
		if (velocity != FVector::ZeroVector) IdleUnlock();
		Super::SetPhysicsLinearVelocity(velocity, addToCurrent, boneName);
	}

	// Set the physics angular velocity of the vehicle.
	virtual void SetPhysicsAngularVelocityInRadians(FVector angularVelocity, bool addToCurrent = false, FName boneName = NAME_None) override
	{
		if (angularVelocity != FVector::ZeroVector) IdleUnlock();
		Super::SetPhysicsAngularVelocityInRadians(angularVelocity, addToCurrent, boneName);
	}

	// Add a force to the vehicle.
	virtual void AddForce(FVector force, FName boneName = NAME_None, bool accelerationChange = false) override
	{
		IdleUnlock();
		Super::AddForce(force, boneName, accelerationChange);
	}

	// Add a force to the vehicle at a particular location in world space.
	virtual void AddForceAtLocation(FVector force, FVector location, FName boneName = NAME_None) override
	{
		IdleUnlock();
		Super::AddForceAtLocation(force, location, boneName);
	}

	// Add a force to the vehicle at a particular location in local space.
	virtual void AddForceAtLocationLocal(FVector force, FVector location, FName boneName = NAME_None) override
	{
		IdleUnlock();
		Super::AddForceAtLocationLocal(force, location, boneName);
	}

	// Add a force to the vehicle, originating from the origin in world space.
	virtual void AddRadialForce(FVector origin, float radius, float strength, enum ERadialImpulseFalloff falloff, bool accelerationChange = false) override
	{
		IdleUnlock();
		Super::AddRadialForce(origin, radius, strength, falloff, accelerationChange);
	}

	// Add an impulse to the vehicle. Good for one time instant burst.
	virtual void AddImpulse(FVector impulse, FName boneName = NAME_None, bool velocityChange = false) override
	{
		IdleUnlock();
		Super::AddImpulse(impulse, boneName, velocityChange);
	}

	// Add an impulse to the vehicle at a specific location.
	virtual void AddImpulseAtLocation(FVector impulse, FVector location, FName boneName = NAME_None) override
	{
		IdleUnlock();
		Super::AddImpulseAtLocation(impulse, location, boneName);
	}

	// Add an impulse to the vehicle, radiating out from the origin in world space.
	virtual void AddRadialImpulse(FVector origin, float radius, float strength, enum ERadialImpulseFalloff falloff, bool velocityChange = false) override
	{
		IdleUnlock();
		Super::AddRadialImpulse(origin, radius, strength, falloff, velocityChange);
	}

	// Set the physics linear velocity of the vehicle.
	virtual void SetAllPhysicsLinearVelocity(FVector velocity, bool addToCurrent = false) override
	{ SetPhysicsLinearVelocity(velocity, addToCurrent); }

	// Add a rotational impulse to this body in radians.
	virtual void AddAngularImpulseInRadians(FVector angularImpulse, FName boneName = NAME_None, bool velocityChange = false) override
	{
		IdleUnlock();
		Super::AddAngularImpulseInRadians(angularImpulse, boneName, velocityChange);
	}

	// Set the physics location and quaternion of the vehicle.
	void SetPhysicsLocationAndQuaternionSubstep(const FVector& location, const FQuat& rotation);

	// Set the physics mass and inertia tensor of the vehicle.
	void SetPhysicsMassAndInertiaTensorSubstep(float mass, const FVector& inertiaTensor);

	// Set the physics linear velocity of the vehicle.
	void SetPhysicsLinearVelocitySubstep(FVector velocity, bool addToCurrent = false, const FName& boneName = NAME_None);

	// Set the physics angular velocity of the vehicle.
	void SetPhysicsAngularVelocityInRadiansSubstep(FVector angularVelocity, bool addToCurrent = false, const FName& boneName = NAME_None);

	// Set the physics angular velocity of the vehicle.
	void SetPhysicsAngularVelocityInDegreesSubstep(const FVector& angularVelocity, bool addToCurrent = false, const FName& boneName = NAME_None)
	{ SetPhysicsAngularVelocityInRadiansSubstep(FMath::DegreesToRadians(angularVelocity), addToCurrent, boneName); }

	// Add a force to the vehicle.
	void AddForceSubstep(FVector force, const FName& boneName = NAME_None, bool accelerationChange = false);

	// Add a force to the vehicle at a particular location in world space.
	void AddForceAtLocationSubstep(const FVector& force, const FVector& location, const FName& boneName = NAME_None);

	// Add a force to the vehicle at a particular location in local space.
	void AddForceAtLocationLocalSubstep(const FVector& force, const FVector& location, const FName& boneName = NAME_None);

	// Add a force to the vehicle, originating from the origin in world space.
	void AddRadialForceSubstep(const FVector& origin, float radius, float strength, enum ERadialImpulseFalloff falloff, bool accelerationChange = false);

	// Add an impulse to the vehicle. Good for one time instant burst.
	void AddImpulseSubstep(const FVector& impulse, const FName& boneName = NAME_None, bool velocityChange = false);

	// Add an impulse to the vehicle at a specific location.
	void AddImpulseAtLocationSubstep(const FVector& impulse, const FVector& location, const FName& boneName = NAME_None);

	// Add an impulse to the vehicle, radiating out from the origin in world space.
	void AddRadialImpulseSubstep(const FVector& origin, float radius, float strength, enum ERadialImpulseFalloff falloff, bool velocityChange = false);

	// Add a rotational impulse to this body in radians.
	void AddAngularImpulseInRadiansSubstep(FVector angularImpulse, FName boneName = NAME_None, bool velocityChange = false);

	// Add a rotational impulse to this body in degrees.
	void AddAngularImpulseInDegreesSubstep(FVector angularImpulse, FName boneName = NAME_None, bool velocityChange = false)
	{ AddAngularImpulseInRadiansSubstep(FMath::DegreesToRadians(angularImpulse), boneName, velocityChange); }

	// Set the physics linear velocity of the vehicle.
	void SetAllPhysicsLinearVelocitySubstep(const FVector& velocity, bool addToCurrent = false)
	{ SetPhysicsLinearVelocitySubstep(velocity, addToCurrent); }

	// Is the vehicle idle?
	bool IsIdle() const
	{ return IdleLocked > 0; }

	// Is the vehicle idle and locked?
	bool IsIdleLocked() const
	{ return IdleLocked >= LockFrames && MainFrameCounter == 0; }

	// Unlock the idle state.
	void IdleUnlock()
	{ IdleLocked = 0; MainFrameCounter = 2; }

	// Idle lock the vehicle to a specific location and rotation.
	void IdleAt(const FVector& location, const FQuat& rotation)
	{
		IdleLocked = 1;
		IdleLocation = location;
		IdleRotation = rotation;
	}

	// Update the idle lock for the vehicle.
	bool UpdateIdleLock(bool fromPhysicsSubStep)
	{
		if (fromPhysicsSubStep == true)
		{
			if (IdleLocked > 0 &&
				IdleLocked < LockFrames)
			{
				IdleLocked++;
			}
		}
		else if (MainFrameCounter > 0)
		{
			MainFrameCounter--;
		}

		return IsIdleLocked();
	}

	// Get the idle location for the vehicle.
	const FVector& GetIdleLocation() const
	{ return IdleLocation; }

	// Get the idle rotation for the vehicle.
	const FQuat& GetIdleRotation() const
	{ return IdleRotation; }

private:

	// The handle of the physics actor.
	FPhysicsActorHandle ActorHandle;

	// The number frames we have to be idle for before we lock the vehicle.
	const int32 LockFrames = 4;

	// The number of frames we've been idle for.
	int32 IdleLocked = 0;

	// Counter for updates from the main thread following an idle unlock.
	int32 MainFrameCounter = 0;

	// The location we're locked at idle.
	FVector IdleLocation = FVector::ZeroVector;

	// The rotation we're locked at idle.
	FQuat IdleRotation = FQuat::Identity;
};

#pragma endregion MinimalVehicle
