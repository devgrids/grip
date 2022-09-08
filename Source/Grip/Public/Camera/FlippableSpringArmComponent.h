/**
*
* Flippable spring arm implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Provides a spring arm for a camera that works well with flippable vehicles and
* contains a number of improvements over and above the standard engine spring arm.
* It doesn't care if the vehicle it's linked to isn't flippable, it doesn't matter.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "system/mathhelpers.h"
#include "flippablespringarmcomponent.generated.h"

/**
* Structure describing the camera offset from its target.
***********************************************************************************/

USTRUCT(BlueprintType)
struct FCameraOffset
{
	GENERATED_USTRUCT_BODY()

public:

	FCameraOffset() = default;

	FCameraOffset(float xoffset, float zoffset, float minDownAngle, float maxDownAngle, float lagRatio)
		: XOffset(xoffset)
		, ZOffset(zoffset)
		, MinDownAngle(minDownAngle)
		, MaxDownAngle(maxDownAngle)
		, LagRatio(lagRatio)
	{ }

	// Interpolate the spring arm between offsets.
	void InterpEaseInOut(const FCameraOffset& from, const FCameraOffset& to, float time, float power)
	{
		XOffset = FMath::InterpEaseInOut(from.XOffset, to.XOffset, time, power);
		ZOffset = FMath::InterpEaseInOut(from.ZOffset, to.ZOffset, time, power);
		MinDownAngle = FMath::InterpEaseInOut(from.MinDownAngle, to.MinDownAngle, time, power);
		MaxDownAngle = FMath::InterpEaseInOut(from.MaxDownAngle, to.MaxDownAngle, time, power);
		LagRatio = FMath::InterpEaseInOut(from.LagRatio, to.LagRatio, time, power);
	}

	// How far in front of the target the camera should be positioned.
	UPROPERTY(EditAnywhere, Category = Offset)
		float XOffset = -500.0f;

	// How far above the target the camera should be positioned.
	UPROPERTY(EditAnywhere, Category = Offset)
		float ZOffset = 200.0f;

	// How much to angle the camera down towards the vehicle in degrees, when moving at high speed.
	UPROPERTY(EditAnywhere, Category = Offset)
		float MinDownAngle = 3.0f;

	// How much to angle the camera down towards the vehicle in degrees, when static.
	UPROPERTY(EditAnywhere, Category = Offset)
		float MaxDownAngle = 5.0f;

	// How much lag should be applied at this offset.
	UPROPERTY(EditAnywhere, Category = Offset)
		float LagRatio = 1.0f;
};

/**
* Class for a spring arm component suitable for a flippable vehicle.
***********************************************************************************/

UCLASS(ClassGroup = Camera, HideCategories = (Transform), meta = (BlueprintSpawnableComponent))
class GRIP_API UFlippableSpringArmComponent : public USceneComponent
{
	GENERATED_BODY()

public:

	// Construct a flippable spring arm component.
	UFlippableSpringArmComponent();

	// Natural length of the spring arm when there are no collisions.
	UPROPERTY(EditAnywhere, Category = Arm)
		TArray<FCameraOffset> CameraOffsets;

	// How big should the query probe sphere be (in unreal units).
	UPROPERTY(EditAnywhere, Category = Arm)
		float ProbeSize = 10.0f;

	// What yaw extension to use?
	UPROPERTY(EditAnywhere, Category = Arm)
		float DriftYawExtension = -1.0f;

	// Controls how quickly camera reaches target pitch angle when the vehicle is on the ground.
	UPROPERTY(EditAnywhere, Category = Lag)
		float CameraPitchLagRatio = 0.98f;

	// Controls how quickly camera reaches target yaw angle when the vehicle is on the ground.
	UPROPERTY(EditAnywhere, Category = Lag)
		float CameraYawLagRatio = 0.90f;

	// Controls how quickly camera reaches target roll angle when the vehicle is on the ground.
	UPROPERTY(EditAnywhere, Category = Lag)
		float CameraRollLagRatio = 0.95f;

	// The amount of forward adjustment to make to compensate for increasing FOV.
	UPROPERTY(EditAnywhere, Category = Camera)
		float FieldOfViewCompensation = 0.8f;

	// Amplitude to shake the camera with high speed.
	UPROPERTY(EditAnywhere, Category = Camera)
		float SpeedShakeAmplitude = 1.0f;

	// Frequency to shake the camera with high speed.
	UPROPERTY(EditAnywhere, Category = Camera)
		float SpeedShakeFrequency = 1.0f;

	// The name of the socket at the end of the spring arm (looking back towards the spring arm origin)
	static const FName SocketName;

#pragma region VehicleSpringArm

public:

	// Shake the camera with high speed.
	float SpeedShakeAmount = 0.0f;

	// The speed of the camera shake, or its frequency of shaking motion.
	float SpeedShakeSpeed = 1.0f;

	// The amount of FOV boost being applied in degrees.
	float FieldOfViewBias = 0.0f;

	// The ratio of FOV being applied, increasing to 1 with speed and therefore the maximum FOV.
	float FieldOfViewProportion = 0.0f;

	// Registration of the component.
	virtual void OnRegister() override;

	// Do the regular update tick.
	virtual void TickComponent(float deltaSeconds, enum ELevelTick tickType, FActorComponentTickFunction* thisTickFunction) override;

	// Does this component have any sockets?
	virtual bool HasAnySockets() const override
	{ return true; }

	// Get a transform for the socket the spring arm is exposing.
	virtual FTransform GetSocketTransform(FName InSocketName, ERelativeTransformSpace TransformSpace = RTS_World) const override;

	// Query the supported sockets.
	virtual void QuerySupportedSockets(TArray<FComponentSocketDescription>& OutSockets) const override
	{ new (OutSockets)FComponentSocketDescription(SocketName, EComponentSocketType::Socket); }

	// Ease the camera in toward the target.
	void CameraIn();

	// Ease the camera out away from the target.
	void CameraOut();

	// Set the camera to an offset from the target.
	void CameraAt(int32 index);

	// Use the rear view camera.
	void RearViewCamera(bool immediately = false)
	{ TargetUserYawAngle = 180.0f; YawActionOverride = true; if (immediately == true) CurrentUserYawAngle = TargetUserYawAngle; }

	// Use the rear view camera.
	void LeftViewCamera(bool immediately = false)
	{ TargetUserYawAngle = -90.0f; YawActionOverride = true; if (immediately == true) CurrentUserYawAngle = TargetUserYawAngle; }

	// Use the rear view camera.
	void RightViewCamera(bool immediately = false)
	{ TargetUserYawAngle = +90.0f; YawActionOverride = true; if (immediately == true) CurrentUserYawAngle = TargetUserYawAngle; }

	// Use the front view camera.
	void FrontViewCamera(bool immediately = false)
	{ TargetUserYawAngle = 0.0f; YawActionOverride = false; if (immediately == true) CurrentUserYawAngle = TargetUserYawAngle; }

	// Looking forwards or backwards.
	void LookForwards(float val, float deadZone)
	{ LookingForwards = val; LookingDeadZone = FMath::Max(0.25f, deadZone); }

	// Looking left or right.
	void LookSideways(float val, float deadZone)
	{ LookingSideways = val; LookingDeadZone = FMath::Max(0.25f, deadZone); }

	// Reset the smoothing on the spring arm movement so the next update is sharp.
	void ResetSmoothing()
	{ SmoothingReset = true; }

	// Query parameters used for collision testing to prevent the camera clipping into geometry.
	FCollisionQueryParams& GetClippingQueryParams()
	{ return ClippingQueryParams; }

	// Is the camera using cockpit view?
	bool IsCockpitView() const
	{ return CameraOffsetIndex == CameraOffsets.Num(); }

	// Is the camera using bumper view?
	bool IsBumperView() const
	{ return CameraOffsetIndex == CameraOffsets.Num() - 1; }

private:

	// Following mode for the spring arm.
	enum class EFollowingMode : uint8
	{
		// The normal mode used during general driving.
		Normal,

		// When airborne.
		Airborne,

		// When crashed.
		Crashed,

		Num
	};

	// Update the arm to the desired properties from a given transform.
	void UpdateDesiredArmProperties(const FTransform& vehicleTransform, FRotator vehicleRotation, float deltaSeconds);

	// Update the arm to the desired properties.
	void UpdateDesiredArmProperties(const FTransform& vehicleTransform, FRotator vehicleRotation, bool doClippingCheck, bool doLocationLag, bool doRotationLag, float deltaSeconds);

	// Get the smoothed rotation, always taking the shortest path if pre-normalized.
	FRotator GetSmoothedRotation(FRotator previous, FRotator next, float deltaSeconds, float zoomRatio) const
	{ return FMathEx::GetSmoothedRotation(previous, next, deltaSeconds, YawLagRatio * zoomRatio, PitchLagRatio * zoomRatio, RollLagRatio * zoomRatio); }

	// Setup the camera offsets for the spring arm.
	void SetupCameraOffsets(int32 lastCameraOffsetIndex);

	// Update the rotation offset, used to emphasize drifting.
	void UpdateRotationOffset(float deltaSeconds, class ABaseVehicle* vehicle, float lagRatio);

	// Get the general following transition ratio.
	float GetFollowingTransitionRatio() const
	{ return (ThisModeTransitionTime == 0.0f) ? 1.0f : FMath::Min(FollowingModeTime / ThisModeTransitionTime, 1.0f); }

	// Get the crashed transition ratio.
	float GetCrashedTransitionRatio() const
	{ if (FollowingMode == EFollowingMode::Crashed) return GetFollowingTransitionRatio(); else if (FromFollowingMode == EFollowingMode::Crashed) return 1.0f - GetFollowingTransitionRatio(); else return 0.0f; }

	// Fixup a couple of angles so that they interpolate the shortest distance between each other.
	void ModifyRotationBasis(float& fromAngle, float& toAngle) const;

	// Fixup a couple of rotations so that they interpolate the shortest distance between each other.
	void ModifyRotationBasis(FRotator& fromRotation, FRotator& toRotation);

	// Make the arm offset in world space from a particular following mode.
	FVector MakeArmOffset(FCameraOffset& cameraOffset, FRotator& rotation, EFollowingMode followingMode, bool includeVerticalOffset) const;

	// Make the arm root as a point on the vehicle to clip towards, but never past.
	FVector MakeArmRoot(const FVector& attachmentRoot, const FVector& armOffset, bool flipped);

	// Is the owner of this vehicle being watched in any viewport?
	bool OwnerIsBeingWatched() const;

	// Clip the spring arm against other vehicles.
	bool ClipAgainstVehicles(const FVector& start, FVector& end) const;

	// Cached component-space socket location, used for transmitting hierarchy down to the camera.
	FVector RelativeSocketLocation;

	// Cached component-space socket rotation, used for transmitting hierarchy down to the camera.
	FQuat RelativeSocketRotation;

	// Location variables in world space.
	FVector TargetLocation = FVector::ZeroVector;
	FVector CurrentLocation = FVector::ZeroVector;

	// Rotation variables in world space.
	FRotator TargetRotation = FRotator::ZeroRotator;
	FRotator CurrentRotation = FRotator::ZeroRotator;

	// The index of the length away from the target are we set to.
	int32 CameraOffsetIndex = 1;

	// The length away from the camera offset we are interpolating from.
	FCameraOffset CameraOffsetFrom;

	// The length away from the camera offset we are interpolating to.
	FCameraOffset CameraOffsetTo;

	// The interpolation time for the length away from the camera offset we are using.
	float CameraOffsetTime = 1.0f;

	// The current yaw lag ratio.
	float YawLagRatio = 0.0f;

	// The current pitch lag ratio.
	float PitchLagRatio = 0.0f;

	// The current roll lag ratio.
	float RollLagRatio = 0.0f;

	// Is the yaw being overridden with a user yaw angle?
	bool YawActionOverride = false;

	// If yaw is being overridden, what is the target user yaw angle?
	float TargetUserYawAngle = 0.0f;

	// If yaw is being overridden, what is the current user yaw angle?
	float CurrentUserYawAngle = 0.0f;

	// How much to look forwards (when not being overridden by a user yaw angle).
	float LookingForwards = 0.0f;

	// How much to look sideways (when not being overridden by a user yaw angle).
	float LookingSideways = 0.0f;

	// The dead zone to use for looking around with the camera.
	float LookingDeadZone = 0.0f;

	// Timer for swiftly changing roll.
	float SpeedRollTimer = 0.0f;

	// Timer for camera shake with high speed.
	float SpeedShakeTimer = 0.0f;

	// How much to angle the camera down towards the vehicle in degrees.
	float DownAngle = 0.0f;

	// Reset the smoothing on the next update.
	bool SmoothingReset = true;

	// Is the spring arm attached to the vehicle body (with animated adjustments) or the vehicle root?
	bool BodyAttachment = false;

	// An offset to the rotation, which is smoothed from frame to frame.
	FRotator RotationOffset = FRotator::ZeroRotator;

	// Oscillator used for speed shake.
	FMathEx::FOscillator SpeedShakeX = FMathEx::FOscillator(23.0f, 12.0f, false);

	// Oscillator used for speed shake.
	FMathEx::FOscillator SpeedShakeY = FMathEx::FOscillator(29.0f, 10.0f, false);

	// Query parameters used for collision testing to prevent the camera clipping into geometry.
	FCollisionQueryParams ClippingQueryParams = FCollisionQueryParams("SpringArmClipping", true);

	// The last following mode of the spring arm to its parent.
	EFollowingMode FromFollowingMode = EFollowingMode::Normal;

	// The following mode of the spring arm to its parent.
	EFollowingMode FollowingMode = EFollowingMode::Normal;

	// How long that following mode has been detected for.
	float FollowingModeTime = 0.0f;

	// The following mode of the spring arm to its parent we've detected on this frame.
	EFollowingMode ToFollowingMode = EFollowingMode::Normal;

	// How long that following mode has been detected for.
	float ToFollowingModeTime = 0.0f;

	// The transition time for this particular mode transition.
	float ThisModeTransitionTime = 0.0f;

	// The last distance the camera was clipped from its parent, for smoothing,
	float LastClippingDistance = 0.0f;

	// Direction towards which the vehicle is supposed to be heading to.
	FVector TargetVehicleHeading = FVector::ZeroVector;

	// Is the parent vehicle?
	bool Airborne = false;

	// How long as it been continuously airborne or grounded for?
	float ContactModeTime = 0.0f;

	// The last launch direction observed in the parent vehicle.
	FVector LaunchDirection = FVector::ZeroVector;

	// Timer used for slipping the launch direction.
	float LaunchDirectionFlipTime = 0.0f;

	// The airborne heading recorded when speed was high enough.
	FRotator AirborneVehicleHeading = FRotator::ZeroRotator;

	// Lengths for computing a rotation transition between following modes.
	FVector FollowingModeVectors[(int32)EFollowingMode::Num];

	// Rotators for computing a rotation transition between following modes.
	FRotator LastRotations[(int32)EFollowingMode::Num][2];
	FRotator Rotations[(int32)EFollowingMode::Num];
	FRotator SmoothedRotations[(int32)EFollowingMode::Num];

	// Same as SmoothedRotations but with a basis that is intelligently interpolated.
	FRotator TransitionRotations[(int32)EFollowingMode::Num][2];

	// The current relative position of the arm root to the parent vehicle.
	FVector ArmRoot = FVector::ZeroVector;

	// Which piece of the code was used to calculate the current position of the arm root.
	int32 ArmRootMode = 1;

	// A timer used to merge air to ground rotations for smooth landing.
	float AirToGroundTime = 10.0f;

	// A time for measuring contact points when airborne.
	float NoAirborneContactTime = 0.0f;

	// Rotations for ameliorating the vertical direction problem.
	FQuat LastGoodVehicleRotation = FQuat::Identity;
	FQuat NinetyDegreeVehicleRotation = FQuat::Identity;

	// Can the owner of the spring arm not see the vehicle?
	bool BaseOwnerNoSee = false;

	// The normal time we use for transitions between following modes.
	const float ModeTransitionTime = 2.0f;

	// Orbit angle on the horizontal plane.
	float OrbitHor = 0.0f;

	// Orbit angle on the vertical plane.
	float OrbitVer = 0.0f;

#pragma endregion VehicleSpringArm

	friend class ADebugRaceCameraHUD;
	friend class ADebugVehicleHUD;
};
