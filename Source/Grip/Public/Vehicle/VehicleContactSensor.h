/**
*
* Wheel contact sensor implementation, use for wheels attached to vehicles.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Contact sensors provide information about nearest surface contacts for a wheel.
* They're paired for flippable vehicles so that we can detect contact but beneath
* and above any given wheel. They also provide suspension for standard vehicles
* and the hovering ability for antigravity vehicles.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"

#pragma region VehicleContactSensors

#include "system/timesmoothing.h"

class ABaseVehicle;
enum class EGameSurface : uint8;

/**
* A structure used to implement a vehicle contact sensor. All of the nearest
* contact sensing and suspension implementation resides in here.
***********************************************************************************/

struct FVehicleContactSensor
{
public:

	// Setup a new sensor.
	void Setup(ABaseVehicle* vehicle, int32 alignment, float side, float startOffset, float wheelWidth, float wheelRadius, float restingCompression);

	// Do the regular update tick.
	void Tick(float deltaTime, UWorld* world, const FTransform& transform, const FVector& startPoint, const FVector& direction, bool updatePhysics, bool estimate, bool calculateIfUpward);

	// Calculate the nearest contact point of the sensor in world space.
	void CalculateContactPoint(float deltaTime, UWorld* world, const FVector& startPoint, const FVector& direction, bool updatePhysics, bool estimate);

	// Get the end point (the outer edge of the wheel) in world space.
	FVector GetEndPoint() const
	{ return (IsInContact() == false) ? GetRestingEndPoint() : EndPoint; }

	// Get the point of nearest geometry in world space.
	const FVector& GetNearestContactPoint() const
	{ return NearestContactPoint; }

	// Get the normal of nearest geometry in world space.
	const FVector& GetNearestContactNormal() const
	{ return NearestContactNormal; }

	// Get the distance of the nearest geometry.
	float GetNearestContactPointDistance() const
	{ return SurfaceDistance; }

	// Get the distance of the nearest geometry.
	float GetNearestContactPointDistanceFromTire() const
	{ return (NearestContactPoint - StartPoint).Size(); }

	// Get the end point (the outer edge of the wheel) when at rest in world space.
	FVector GetRestingEndPoint() const
	{ return SensorPositionFromLength(WheelRadius + HoverDistance); }

	// Get the direction of the sensor in world space.
	FVector GetDirection() const;

	// Get the amount of suspension spring extension (or offset of the wheel).
	float GetExtension() const;

	// Is the sensor itself in contact with a surface, that is the contact is within the reach of the wheel?
	bool IsInContact() const
	{ return InContact; }

	// Is the sensor itself in contact with a surface, that is the contact is within the reach of the hovering effectiveness?
	bool IsInEffect() const
	{ return InEffect; }

	// Is the nearest contact point valid?
	bool HasNearestContactPoint(const FVector& wheelVelocity, float contactSeconds) const;

	// Get the compression of the suspension in cms.
	float GetCompression() const
	{ return Compression; }

	// Get the resting compression of the suspension.
	float GetRestingCompression()
	{ return RestingCompression; }

	// Set the resting compression of the suspension from a known value.
	void SetRestingCompression(float compression)
	{ RestingCompression = compression; }

	// Get a normalized compression ratio of the suspension between 0 and 10, 1 being resting under static weight.
	// Up to 2 is normally the maximum range.
	float GetNormalizedCompression() const
	{ return GetNormalizedCompression(Compression); }

	// Get the alignment of the sensor.
	float GetAlignment() const
	{ return Alignment; }

	// Get the side of the sensor.
	float GetSide() const
	{ return FMathEx::UnitSign(Side); }

	// Get the length of the ray casting down the sensor to detect driving surfaces.
	float GetSensorLength() const;

	// Get the distance from the top of the sensor to the driving surface in cms, 0 = no surface.
	float GetSurfaceDistance() const
	{ return SurfaceDistance; }

	// Was the suspension just compressed down hard?
	bool WasCompressedHard() const
	{ return CompressedHard; }

	// Spawn an effect based on hard compression and therefore an impact has occurred.
	void SpawnCompressionEffect()
	{ CompressionEffectRequired = IsInContact(); CompressionEffectLocation = GetNearestContactPoint(); }

	// Is a compression effect required at this point?
	bool IsCompressionEffectRequired(FVector& location)
	{ bool result = CompressionEffectRequired; CompressionEffectRequired = false; location = CompressionEffectLocation; return result; }

	// Get the hit result of the last contact.
	const FHitResult& GetHitResult() const
	{ return HitResult; }

	// Get the surface material of the last contact.
	const UPhysicalMaterial* GetSurfaceMaterial() const
	{ return (IsInContact() == true && GRIP_POINTER_VALID(HitResult.PhysMaterial) == true) ? &*HitResult.PhysMaterial : nullptr; }

	// Get the game surface of the last contact.
	EGameSurface GetGameSurface() const;

	// Has the sensor detected a valid driving surface?
	bool HasValidDrivingSurface(const FVector& wheelVelocity, float contactSeconds) const;

	// Sweeps along sensor direction to see if the suspension spring needs to compress.
	// Returned collision time is normalized.
	bool GetCollision(UWorld * world, const FVector & start, const FVector & end, float& time, FHitResult & hitResult, bool estimate);

	// Apply the suspension spring force to the vehicle.
	void ApplyForce(const FVector& atPoint) const;

	// Get the width of the suspension sweep in cms.
	float GetSweepWidth() const;

	// Get the amount of time a wheel has not been in contact with a driving surface.
	float GetNonContactTime() const
	{ return NonContactTime; }

	// Reset any contact following a teleport of some kind.
	void ResetContact()
	{ EstimateContact = InContact = NearestContactValid = false; }

	// Is the suspension at rest and not changing compression?
	bool IsAtRest() const
	{ return FMath::Abs(CompressionList.GetMeanValue() - CompressionList.GetLastValue()) < 0.1f; }

	// The maximum size of the force applied for any sub-step during the last tick (for visualization purposes).
	float ForceApplied = 0.0f;

	// The force applied during the last tick.
	FVector ForceToApply = FVector::ZeroVector;

	// The amount of tilting to perform on the sensor for antigravity vehicles.
	float TiltScale = 0.0f;

	// When stuck on a surface and can't transition to the next sideways, then offset the tilt to try to ease that transition.
	// This is kind of a hack to make antigravity vehicles easier to control.
	float OutboardOffset = 0.0f;

	// The width of the wheel.
	float WheelWidth = 0.0f;

	// The radius of the wheel.
	float WheelRadius = 0.0f;

private:

	// Computes new suspension spring compression and force.
	FVector ComputeNewSpringCompressionAndForce(const FVector& end, float deltaTime);

	// Given a length, returns the point along the sensor that is length units away from the sensor start.
	FVector SensorPositionFromLength(float length) const
	{ return StartPoint + length * GetDirection(); }

	// Get a normalized compression ratio of the suspension spring between 0 and 10, 1 being resting under static weight.
	float GetNormalizedCompression(float value) const;

	// The vehicle to which the sensor is connected.
	ABaseVehicle* Vehicle = nullptr;

	// The alignment of the sensor to the vehicle, 1 = up and -1 = down.
	float Alignment = 0.0f;

	// The side of the sensor to the vehicle, negative = left and positive = right.
	float Side = -1.0f;

	// The sampled Compression value when the vehicle is known to be at rest.
	float RestingCompression = 0.0f;

	// The offset in Z of the start position from the bone position of the wheel.
	float StartOffset = 0.0f;

	// How much compression the suspension is experiencing.
	float Compression = 0.0f;

	// The start point of the sensor in world space.
	FVector StartPoint = FVector::ZeroVector;

	// The direction of the parent vehicle in world space.
	FVector Direction = FVector::ZeroVector;

	// The end point of the sensor (the outer edge of the wheel) in world space.
	FVector EndPoint = FVector::ZeroVector;

	// The nearest contact point of the sensor (the outer edge of the wheel) in world space.
	FVector NearestContactPoint = FVector::ZeroVector;

	// The nearest contact normal of the sensor (the outer edge of the wheel) in world space.
	FVector NearestContactNormal = FVector::ZeroVector;

	// The hit result of the last contact.
	FHitResult HitResult;

	// The last contact time given from a collision test.
	float EstimateTime = 0.0f;

	// The distance from start to contact point when setting up the estimate.
	float EstimateDistance = 0.0f;

	// Will be "under" the sensor.
	FVector EstimateContactPoint = FVector::ZeroVector;

	// Almost certainly "towards" the sensor.
	FVector EstimateContactNormal = FVector::ZeroVector;

	// Estimate the next surface contact rather than ray-casting for it?
	bool EstimateContact = false;

	// Is the sensor in contact with a surface?
	bool InContact = false;

	// Is the suspension in effect?
	bool InEffect = false;

	// Is the nearest contact point valid?
	bool NearestContactValid = false;

	// The amount of time the sensor has been out of contact.
	float NonContactTime = 0.0f;

	// The distance from the top of the sensor to the driving surface in cms, 0 = no surface.
	float SurfaceDistance = 0.0f;

	// The distance from the bottom of the tire to the driving surface in cms, < 0 = no surface.
	float SurfaceDistanceFromTire = -1.0f;

	// Is the suspension compressed down hard?
	bool CompressedHard = false;

	// Is the suspension compressing down hard?
	bool CompressingHard = false;

	// Is a compression effect required for the suspension.
	bool CompressionEffectRequired = false;

	// The location of the compression effect in world space.
	FVector CompressionEffectLocation = FVector::ZeroVector;

	// The current, intended resting hovering distance from the driving surface.
	// Antigravity vehicles only.
	float HoverDistance = 0.0f;

	// The current, maximum contact hovering distance from the driving surface.
	// The further away from the surface you are, the less grip effect you will receive.
	// Antigravity vehicles only.
	float HoverContactDistance = 0.0f;

	// Values for suspension compression over time.
	FTimedFloatList CompressionList = FTimedFloatList(1, 10);

	// The shape to be used for performing a sensor sweep.
	FCollisionShape SweepShape;
};

#pragma endregion VehicleContactSensors
