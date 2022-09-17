/**
*
* Base vehicle implementation.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* The main vehicle class, containing almost all the meat of the vehicle
* implementation, both standard and flippable.
*
***********************************************************************************/

#include "vehicle/basevehicle.h"
#include "components/inputcomponent.h"
#include "components/widgetcomponent.h"
#include "effects/vehicleimpacteffect.h"
#include "gamemodes/playgamemode.h"
#include "runtime/umg/public/umg.h"
#include "game/globalgamestate.h"
#include "ui/hudwidget.h"
#include "gamemodes/menugamemode.h"
#include "ai/pursuitsplineactor.h"
#include "blueprint/widgetblueprintlibrary.h"
#include "pickups/shield.h"
#include "pickups/turbo.h"
#include "camera/camerapointcomponent.h"

#pragma region BlueprintAssets

TSubclassOf<AGatlingGun> ABaseVehicle::Level1GatlingGunBlueprint = nullptr;
TSubclassOf<AGatlingGun> ABaseVehicle::Level2GatlingGunBlueprint = nullptr;
TSubclassOf<AHomingMissile> ABaseVehicle::Level1MissileBlueprint = nullptr;
TSubclassOf<AHomingMissile> ABaseVehicle::Level2MissileBlueprint = nullptr;
TSubclassOf<AShield> ABaseVehicle::Level1ShieldBlueprint = nullptr;
TSubclassOf<AShield> ABaseVehicle::Level2ShieldBlueprint = nullptr;
TSubclassOf<ATurbo> ABaseVehicle::Level1TurboBlueprint = nullptr;
TSubclassOf<ATurbo> ABaseVehicle::Level2TurboBlueprint = nullptr;
TSubclassOf<AElectricalBomb> ABaseVehicle::DestroyedElectricalBomb = nullptr;
UParticleSystem* ABaseVehicle::DestroyedParticleSystem = nullptr;
UParticleSystem* ABaseVehicle::ResetEffectBlueprint = nullptr;
UParticleSystem* ABaseVehicle::LaunchEffectBlueprint = nullptr;
UParticleSystem* ABaseVehicle::HardImpactEffect = nullptr;
UParticleSystem* ABaseVehicle::DamageEffect = nullptr;
UParticleSystem* ABaseVehicle::DamageSparks = nullptr;
UMaterialInterface* ABaseVehicle::CockpitGhostMaterial = nullptr;
UMaterialInterface* ABaseVehicle::CheapCameraMaterial = nullptr;
UMaterialInterface* ABaseVehicle::ExpensiveCameraMaterial = nullptr;
USoundCue* ABaseVehicle::TeleportSound = nullptr;
USoundCue* ABaseVehicle::LaunchSound = nullptr;
USoundCue* ABaseVehicle::DestroyedSound = nullptr;
USoundCue* FVehicleHUD::HomingMissileIndicatorSound = nullptr;
USoundCue* FVehicleHUD::HomingMissileIndicatorCriticalSound = nullptr;
USoundCue* FVehicleHUD::PickupChargedSound = nullptr;
USoundCue* FVehicleHUD::PickupChargingSound = nullptr;
USoundCue* FVehicleHUD::PickupNotChargeableSound = nullptr;
USoundCue* FVehicleElimination::AlertSound = nullptr;

#pragma endregion BlueprintAssets

float ABaseVehicle::PickupHookTime = 0.5f;
bool ABaseVehicle::ProbabilitiesInitialized = false;

#pragma region Vehicle

/**
* Construct a base vehicle.
***********************************************************************************/

ABaseVehicle::ABaseVehicle()
{
	{
		static ConstructorHelpers::FObjectFinder<UMaterialInterface> asset(TEXT("Material'/Game/Vehicles/Materials/M_HMDGhostVehicle.M_HMDGhostVehicle'"));
		CockpitGhostMaterial = asset.Object;
	}

	{
		static ConstructorHelpers::FObjectFinder<USoundCue> asset(TEXT("SoundCue'/Game/Audio/Sounds/UI/A_EliminationAlert_Cue.A_EliminationAlert_Cue'"));
		Elimination.AlertSound = asset.Object;
	}

	WheelAssignments.Emplace(FWheelAssignment(("F_L_T"), EWheelPlacement::Front, 80.0f, 85.0f, 2.0f, 10.0f));
	WheelAssignments.Emplace(FWheelAssignment(("F_R_T"), EWheelPlacement::Front, 80.0f, 85.0f, 2.0f, 10.0f));
	WheelAssignments.Emplace(FWheelAssignment(("B_L_T"), EWheelPlacement::Rear, 80.0f, 85.0f, 2.0f, 10.0f));
	WheelAssignments.Emplace(FWheelAssignment(("B_R_T"), EWheelPlacement::Rear, 80.0f, 85.0f, 2.0f, 10.0f));

	// We choose to Tick post-physics because we want to be working with the very latest physics data,
	// and also to help avoid any multi-threading issues that might arise from a vehicle accessing its
	// own member data simultaneously while in the main game thread and the physics sub-step thread.

	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PostPhysics;

	VehicleMesh = CreateDefaultSubobject<UVehicleMeshComponent>(TEXT("VehicleMesh"));

	VehicleMesh->SetCollisionProfileName(UCollisionProfile::Vehicle_ProfileName);
	VehicleMesh->BodyInstance.bSimulatePhysics = true;
	VehicleMesh->BodyInstance.bContactModification = true;
	VehicleMesh->BodyInstance.bNotifyRigidBodyCollision = true;
	VehicleMesh->BodyInstance.bUseCCD = false;
	VehicleMesh->bBlendPhysics = true;

	VehicleMesh->PrimaryComponentTick.TickGroup = PrimaryActorTick.TickGroup;

	SetRootComponent(VehicleMesh);

	SpringArm = CreateDefaultSubobject<UFlippableSpringArmComponent>(TEXT("SpringArm"));
	GRIP_ATTACH(SpringArm, RootComponent, NAME_None);
	SpringArm->PrimaryComponentTick.TickGroup = PrimaryActorTick.TickGroup;

	Camera = CreateDefaultSubobject<URaceCameraComponent>(TEXT("VehicleCamera"));
	GRIP_ATTACH(Camera, SpringArm, UFlippableSpringArmComponent::SocketName);
	Camera->bUsePawnControlRotation = false;
	Camera->PrimaryComponentTick.TickGroup = PrimaryActorTick.TickGroup;

	DamageLight = CreateDefaultSubobject<UPointLightComponent>(TEXT("DamageLight"));
	GRIP_ATTACH(DamageLight, RootComponent, NAME_None);

	DestroyedExplosionForce = CreateDefaultSubobject<URadialForceComponent>(TEXT("DestroyedExplosionForce"));
	DestroyedExplosionForce->bAutoActivate = false;
	GRIP_ATTACH(DestroyedExplosionForce, RootComponent, NAME_None);

	PickedUpEffect = CreateDefaultSubobject<UParticleSystemComponent>(TEXT("PickedUpEffect"));
	PickedUpEffect->bAutoDestroy = false;
	PickedUpEffect->bAutoActivate = false;
	PickedUpEffect->SetHiddenInGame(true);
	GRIP_ATTACH(PickedUpEffect, VehicleMesh, "RootDummy");

	for (int32 i = 0; i < NumDefaultWheels; i++)
	{
		WheelOffsets.Emplace(FVector::ZeroVector);
		WheelRotations.Emplace(FRotator::ZeroRotator);
	}

#if GRIP_ENGINE_PHYSICS_MODIFIED
	OnCalculateCustomPhysics.BindUObject(this, &ABaseVehicle::SubstepPhysics);
#endif // GRIP_ENGINE_PHYSICS_MODIFIED
}

#pragma region APawn

/**
* Setup the player input.
***********************************************************************************/

void ABaseVehicle::SetupPlayerInputComponent(UInputComponent* inputComponent)
{
	int32 localPlayerIndex = DetermineLocalPlayerIndex();

	if (localPlayerIndex >= 0)
	{

#pragma region VehicleControls

		inputComponent->BindAxis("Throttle", this, &ABaseVehicle::Throttle);
		inputComponent->BindAxis("DigitalSteering", this, &ABaseVehicle::DigitalSteering);
		inputComponent->BindAxis("AnalogSteering", this, &ABaseVehicle::AnalogSteering);
		inputComponent->BindAction("Brake", IE_Pressed, this, &ABaseVehicle::HandbrakePressed);
		inputComponent->BindAction("Brake", IE_Released, this, &ABaseVehicle::HandbrakeReleased);
		inputComponent->BindAxis("PitchInput", this, &ABaseVehicle::PitchControl);

#pragma endregion VehicleControls

#pragma region VehicleSpringArm

		inputComponent->BindAxis("LookForwards", this, &ABaseVehicle::LookForwards);
		inputComponent->BindAxis("LookSideways", this, &ABaseVehicle::LookSideways);
		inputComponent->BindAction("CameraIn", IE_Pressed, this, &ABaseVehicle::CameraIn);
		inputComponent->BindAction("CameraOut", IE_Pressed, this, &ABaseVehicle::CameraOut);
		inputComponent->BindAction("LookBack", IE_Pressed, this, &ABaseVehicle::RearViewCamera);
		inputComponent->BindAction("LookBack", IE_Released, this, &ABaseVehicle::FrontViewCamera);
		inputComponent->BindAction("LookLeft", IE_Pressed, this, &ABaseVehicle::LeftViewCamera);
		inputComponent->BindAction("LookLeft", IE_Released, this, &ABaseVehicle::FrontViewCamera);
		inputComponent->BindAction("LookRight", IE_Pressed, this, &ABaseVehicle::RightViewCamera);
		inputComponent->BindAction("LookRight", IE_Released, this, &ABaseVehicle::FrontViewCamera);

#pragma endregion VehicleSpringArm

		APlayerController* controller = Cast<APlayerController>(GetController());

		if (GameMode != nullptr &&
			controller != nullptr)
		{
			GameMode->SetInputOptions(controller);
		}
	}
}

/**
* Do some pre initialization just before the game is ready to play.
***********************************************************************************/

void ABaseVehicle::PreInitializeComponents()
{
	UE_LOG(GripLog, Log, TEXT("ABaseVehicle::PreInitializeComponents"));

	World = GetWorld();
	GameMode = ABaseGameMode::Get(this);
	PlayGameMode = APlayGameMode::Get(this);
	GameState = UGlobalGameState::GetGlobalGameState(this);

	if (VehicleMesh != nullptr)
	{
		PhysicsBody = VehicleMesh->GetBodyInstance();

		if (PhysicsBody != nullptr)
		{
			if (PlayGameMode != nullptr)
			{
				Physics.StockMass = PhysicsBody->GetBodyMass();
				Physics.CurrentMass = Physics.CompressedMass = Physics.StockMass;

				PhysicsBody->PositionSolverIterationCount = 4;
				PhysicsBody->VelocitySolverIterationCount = 1;

				VehicleMesh->SetMassOverrideInKg(NAME_None, Physics.StockMass, true);
				VehicleMesh->SetAngularDamping(0.333f);
			}

			SetupExtraCollision();
		}
	}
}

/**
* Do some post initialization just before the game is ready to play.
***********************************************************************************/

void ABaseVehicle::PostInitializeComponents()
{
	UE_LOG(GripLog, Log, TEXT("ABaseVehicle::PostInitializeComponents"));

	Super::PostInitializeComponents();

	RaceState.HitPoints = 150;
	RaceState.MaxHitPoints = RaceState.HitPoints;

	DamageLight->SetIntensity(0.0f);

	FTransform identity;

	identity.SetIdentity();
	identity.SetScale3D(VehicleMesh->GetComponentTransform().GetScale3D());

	FTransform rootBoneTransform = VehicleMesh->GetBoneTransform(0);

	AttachedEffectsScale = FVector(1.0f, 1.0f, 1.0f) / rootBoneTransform.GetScale3D();

	AI.LastLocation = AI.PrevLocation = GetActorLocation();
	Physics.VelocityData.VelocityDirection = GetActorRotation().Vector();

	// Initial hookup, the absolute nearest point will do.

	int32 numWheels = WheelAssignments.Num();

	if (numWheels != 0)
	{
		WheelOffsets.Empty();
		WheelRotations.Empty();
	}

#pragma region VehicleContactSensors

	// Let's setup the wheels from the wheel bone assignments.

	float frontSum = 0.0f;
	float rearSum = 0.0f;

	Wheels.Wheels.Reserve(numWheels);

	ContactSensorQueryParams.bReturnPhysicalMaterial = true;

	for (const FWheelAssignment& assignment : WheelAssignments)
	{
		FName boneName = assignment.BoneName;
		int32 boneIndex = VehicleMesh->GetBoneIndex(boneName);
		EWheelPlacement placement = assignment.Placement;

		if (boneIndex != INDEX_NONE)
		{
			FVector boneOffset = VehicleMesh->GetBoneTransform(boneIndex, identity).GetLocation();
			FVector standardOffset = FVector(boneOffset.X, boneOffset.Y, 0.0f);
			FVector suspensionForcesOffset = standardOffset;

#pragma region VehicleGrip

			if (TireFrictionModel != nullptr &&
				TireFrictionModel->Model == ETireFrictionModel::Arcade)
			{
				// This standard offset is use purely for the application of grip, in order to bring predictable
				// handling to the vehicles. If we don't do this, we'll have the back-end spin-out when cornering
				// hard for example. Setting the application of grip at relatively fixed offsets around the
				// vehicle helps a lot to reduce unwanted, inconsistent behavior across different vehicles.

				standardOffset = FVector(0.0f, boneOffset.Y, 0.0f);

				if (placement == EWheelPlacement::Rear ||
					placement == EWheelPlacement::Front)
				{
					standardOffset.X = 175.0f * FMathEx::UnitSign(boneOffset.X);
				}
			}

#pragma endregion VehicleGrip

			// Create the wheel from the data we now have.

			FVehicleWheel wheel = FVehicleWheel(boneName, boneOffset, standardOffset, suspensionForcesOffset, placement, assignment.Width, assignment.Radius);

			// Determine where the front and rear axle offsets will end up.

			if (wheel.HasFrontPlacement() == true)
			{
				frontSum += 1.0f;
				Wheels.FrontAxleOffset += boneOffset.X;
			}
			else if (wheel.HasRearPlacement() == true)
			{
				rearSum += 1.0f;
				Wheels.RearAxleOffset += boneOffset.X;
			}

			// Now create the contact sensors for the wheel.

			int32 sensorIndex = 0;

			for (FVehicleContactSensor& sensor : wheel.Sensors)
			{
				sensor.Setup(this, ((sensorIndex++ == 0) ? 1 : -1), boneOffset.Y, assignment.VerticalOffset, assignment.Width, assignment.Radius, assignment.RestingCompression);
			}

			// Add the new wheel with its sensors to our internal list.

			Wheels.Wheels.Emplace(wheel);

			// Create the data required for the animation blueprint.

			WheelOffsets.Emplace(FVector::ZeroVector);
			WheelRotations.Emplace(FRotator::ZeroRotator);
		}
	}

	// Complete the calculation of where the front and rear offsets are, from the average of
	// the wheels attached to those axles.

	if (frontSum != 0.0f)
	{
		Wheels.FrontAxleOffset /= frontSum;
	}

	if (rearSum != 0.0f)
	{
		Wheels.RearAxleOffset /= rearSum;
	}

#pragma endregion VehicleContactSensors

#pragma region VehicleBasicForces

	// Record the total gravity for later to save continually computing it.

	Physics.GravityStrength = FMath::Abs(GetGravityForce(true).Z);

#pragma endregion VehicleBasicForces

	AI.OptimumSpeedExtension = FMath::Max(0.0f, (GripCoefficient - 0.5f) * 2.0f);

	if (PlayGameMode != nullptr &&
		VehicleEngineModel != nullptr)
	{
		float scale = GameState->GeneralOptions.GetEnginePowerScale(GameState->GetDifficultyLevel());

		Propulsion.MaxJetEnginePower = (VehicleEngineModel->JetEnginePower) * scale * PowerCoefficient;
		Propulsion.MaxJetEnginePowerAirborne = VehicleEngineModel->JetEnginePowerAirborne * scale * PowerCoefficient;
	}
}

/**
* Do some initialization when the game is ready to play.
***********************************************************************************/

void ABaseVehicle::BeginPlay()
{
	UE_LOG(GripLog, Log, TEXT("ABaseVehicle::BeginPlay"));

	Super::BeginPlay();

	ProbabilitiesInitialized = false;

	DetermineLocalPlayerIndex();

	CompletePostSpawn();

	TArray<UActorComponent*> components;

	GetComponents(UStaticMeshComponent::StaticClass(), components);

	for (UActorComponent* component : components)
	{
		UStaticMeshComponent* mesh = Cast<UStaticMeshComponent>(component);

		if (mesh != nullptr &&
			mesh->GetName().EndsWith("Rim"))
		{
			mesh->SetForcedLodModel(1);
		}
	}

	GetComponents(UParticleSystemComponent::StaticClass(), components);

	for (UActorComponent* component : components)
	{
		UParticleSystemComponent* particles = Cast<UParticleSystemComponent>(component);

		if (particles->Template != nullptr &&
			particles->Template->GetName().Contains(TEXT("Turbo")) == true)
		{
			TurboParticleSystems.Emplace(particles);
		}
	}

#pragma region VehicleSpringArm

	TArray<int32>& racePositions = GameState->TransientGameState.RaceCameraPositions;

	if (racePositions.IsValidIndex(LocalPlayerIndex) == true)
	{
		SpringArm->CameraAt(racePositions[LocalPlayerIndex]);
	}
	else
	{
		SpringArm->CameraAt(1);
	}

#pragma endregion VehicleSpringArm

	Physics.StartLocation = GetActorLocation();
	Physics.StartRotation = GetActorRotation();

	GetComponents(ULightStreakComponent::StaticClass(), components);

	for (UActorComponent* component : components)
	{
		Cast<ULightStreakComponent>(component)->SetGlobalAmount(0.0f, 0.0f);

		ABaseGameMode::SleepComponent(Cast<ULightStreakComponent>(component));

		LightStreaks.Emplace(Cast<ULightStreakComponent>(component));
	}

	GetComponents(UCameraPointComponent::StaticClass(), components);

	for (UActorComponent* component : components)
	{
		ABaseGameMode::SleepComponent(Cast<UCameraPointComponent>(component));
	}

	static FName rootDummy = FName("RootDummy");

	RootDummyBoneIndex = VehicleMesh->GetBoneIndex(rootDummy);
}

/**
* Do some shutdown when the actor is being destroyed.
***********************************************************************************/

void ABaseVehicle::EndPlay(const EEndPlayReason::Type endPlayReason)
{
	UE_LOG(GripLog, Log, TEXT("ABaseVehicle::EndPlay"));

	if (PlayGameMode != nullptr)
	{
		GRIP_REMOVE_FROM_GAME_MODE_LIST_FROM(Vehicles, PlayGameMode);

		PlayGameMode->RemoveAvoidable(this);
	}

	Super::EndPlay(endPlayReason);
}

/**
* Do the regular update tick, in this case just after the physics has been done.
***********************************************************************************/

void ABaseVehicle::Tick(float deltaSeconds)
{
	Super::Tick(deltaSeconds);

	const FTransform& transform = VehicleMesh->GetComponentTransform();
	FQuat quaternion = transform.GetRotation();
	FVector xdirection = transform.GetUnitAxis(EAxis::X);
	FVector ydirection = transform.GetUnitAxis(EAxis::Y);
	FVector zdirection = transform.GetUnitAxis(EAxis::Z);

	UpdatePhysics(deltaSeconds, transform);

	// Emergency check, should always be a valid pointer for a running game though.

	if (PlayGameMode == nullptr)
	{
		return;
	}

#pragma region VehicleSpringArm

	UpdateCockpitMaterials();

#pragma endregion VehicleSpringArm

	RaceState.Tick(deltaSeconds, PlayGameMode, GameState);

	// If we're now finished playing as a result of that Tick, then hand
	// over to AI control now.

	if (AI.BotDriver == false &&
		RaceState.PlayerCompletionState >= EPlayerCompletionState::Complete)
	{
		SetAIDriver(true);
	}

#pragma region VehicleControls

	InterpolateControlInputs(deltaSeconds);

	UpdateSteering(deltaSeconds, xdirection, ydirection, quaternion);

#pragma endregion VehicleControls

#pragma region VehicleAnimation

	// Update the animated bones, mostly related to having the wheels animate with rolling,
	// steering and suspension movement.

	UpdateAnimatedBones(deltaSeconds, xdirection, ydirection);

#pragma endregion VehicleAnimation

#pragma region VehicleBasicForces

	UpdatePowerAndGearing(deltaSeconds, xdirection, zdirection);

#pragma endregion VehicleBasicForces

	UpdateIdleLock();

	AI.LastVehicleContacts = AI.VehicleContacts;
	AI.LastCollisionBlockage = AI.CollisionBlockage;
	AI.LastHardCollisionBlockage = AI.HardCollisionBlockage;

	AI.VehicleContacts = VehicleUnblocked;
	AI.CollisionBlockage = VehicleUnblocked;
	AI.HardCollisionBlockage = VehicleUnblocked;
}

/**
* Receive hit information from the collision system.
***********************************************************************************/

void ABaseVehicle::NotifyHit(class UPrimitiveComponent* thisComponent, class AActor* other, class UPrimitiveComponent* otherComponent, bool selfMoved, FVector hitLocation, FVector hitNormal, FVector normalForce, const FHitResult& hitResult)
{
	normalForce *= 1.0f / CustomTimeDilation;

	Super::NotifyHit(thisComponent, other, otherComponent, selfMoved, hitLocation, hitNormal, normalForce, hitResult);

	if (hitResult.IsValidBlockingHit() == true)
	{
	}
}

#pragma endregion APawn

#pragma region VehiclePhysics

/**
* Update the physics portion of the vehicle.
***********************************************************************************/

void ABaseVehicle::UpdatePhysics(float deltaSeconds, const FTransform& transform)
{
	// This feels wrong adding custom physics every tick, but it's exactly right.

	PhysicsBody = VehicleMesh->GetBodyInstance();

	if (PhysicsBody != nullptr)
	{
#if GRIP_ENGINE_PHYSICS_MODIFIED
		PhysicsBody->AddCustomPhysics(OnCalculateCustomPhysics);
#else // GRIP_ENGINE_PHYSICS_MODIFIED
		SubstepPhysics(deltaSeconds, PhysicsBody);
#endif // GRIP_ENGINE_PHYSICS_MODIFIED
	}

	if (IsVehicleDestroyed() == true)
	{
		SetActorLocation(Physics.StaticHold.Location, false, nullptr, ETeleportType::TeleportPhysics);
	}

	Wheels.FrontAxlePosition = transform.TransformPosition(FVector(Wheels.FrontAxleOffset, 0.0f, 0.0f));
	Wheels.RearAxlePosition = transform.TransformPosition(FVector(Wheels.RearAxleOffset, 0.0f, 0.0f));

	VehicleClock += deltaSeconds;
	Physics.Drifting.Timer += deltaSeconds;

	if (PlayGameMode != nullptr)
	{
		int32 totalVehicles = PlayGameMode->GetVehicles().Num();

		Clock0p5.Tick(VehicleIndex, totalVehicles);
		Clock0p25.Tick(VehicleIndex, totalVehicles);
		Clock0p1.Tick(VehicleIndex, totalVehicles);

#pragma region VehicleBasicForces

		if (PlayGameMode->PastGameSequenceStart() == false)
		{
			// Lock the vehicle down until the game has started.

			ArrestVehicle();
		}
		else
		{
			Physics.StaticHold.Active = false;
		}

#pragma endregion VehicleBasicForces

	}

	if (Physics.Timing.TickCount > 0)
	{
		Physics.Timing.GeneralTickSum += deltaSeconds;
		Physics.Timing.GeneralTickCount++;

		// If we have an impulse to apply which we've built-up during the physics sub-step
		// then apply it now.

		if (Physics.ApplyImpulse != FVector::ZeroVector)
		{
			VehicleMesh->IdleUnlock();
			VehicleMesh->AddImpulse(Physics.ApplyImpulse);

			Physics.ApplyImpulse = FVector::ZeroVector;

			if (ShieldChargedImpactSound != nullptr)
			{
				AShield* shield = Level2ShieldBlueprint->GetDefaultObject<AShield>();

				ShieldChargedImpactSound = shield->ChargedImpact;
			}

			UGameplayStatics::SpawnSoundAttached(ShieldChargedImpactSound, VehicleMesh, NAME_None, FVector::ZeroVector, EAttachLocation::Type::KeepRelativeOffset);
		}
	}

	ContactPoints[1].Reset();
	ContactForces[1].Reset();

	ContactPoints[1] = ContactPoints[0];
	ContactForces[1] = ContactForces[0];

	ContactPoints[0].Reset();
	ContactForces[0].Reset();
}

/**
* Is the vehicle currently with all wheels off the ground?
***********************************************************************************/

bool ABaseVehicle::IsAirborne(bool ignoreSkipping)
{
	if (ignoreSkipping == false)
	{
		return Physics.ContactData.Airborne;
	}
	else
	{

#pragma region VehicleContactSensors

		for (FVehicleWheel& wheel : Wheels.Wheels)
		{
			// If any wheel is some distance from the ground then return the physics airborne state.

			if (wheel.GetActiveSensor().HasNearestContactPoint(wheel.Velocity, 0.0f) == false ||
				wheel.GetActiveSensor().GetNearestContactPointDistance() > (wheel.Radius + HoverDistance) * 2.0f)
			{
				return Physics.ContactData.Airborne;
			}
		}

		// Otherwise assume we're grounded.

#pragma endregion VehicleContactSensors

		return false;
	}
}

/**
* Setup any additional collision structures for the vehicle.
***********************************************************************************/

void ABaseVehicle::SetupExtraCollision()
{
	if (VehicleMesh != nullptr &&
		PhysicsBody != nullptr)
	{
		const float maxDepenetration = 200.0f;

		PhysicsBody->SetMaxDepenetrationVelocity(maxDepenetration);
		PhysicsBody->SetContactModification(true);

		UPhysicalMaterial* material = PhysicsBody->GetSimplePhysicalMaterial();

#if GRIP_ENGINE_PHYSICS_MODIFIED
		Physics.BodyBounds = PhysicsBody->CalculateLocalBounds(VehicleMesh->GetComponentTransform().GetScale3D());
#else // GRIP_ENGINE_PHYSICS_MODIFIED
		Physics.BodyBounds = FBox(VehicleMesh->Bounds.BoxExtent * -1.0f, VehicleMesh->Bounds.BoxExtent);
#endif // GRIP_ENGINE_PHYSICS_MODIFIED

		material->Friction = 0.0f;
		material->bOverrideFrictionCombineMode = true;
		material->FrictionCombineMode = EFrictionCombineMode::Type::Min;

		material->Restitution = 0.0f;
		material->bOverrideRestitutionCombineMode = true;
		material->RestitutionCombineMode = EFrictionCombineMode::Type::Min;

		FPhysicsInterface::UpdateMaterial(material->GetPhysicsMaterial(), material);

		FVector extent = (VehicleMesh->SkeletalMesh->GetImportedBounds().BoxExtent * 0.5f);

		for (const FWheelAssignment& assignment : WheelAssignments)
		{
			int32 boneIndex = VehicleMesh->GetBoneIndex(assignment.BoneName);

			if (boneIndex != INDEX_NONE)
			{
				FTransform identity;

				identity.SetIdentity();
				identity.SetScale3D(VehicleMesh->GetComponentTransform().GetScale3D());

				FVector boneOffset = VehicleMesh->GetBoneTransform(boneIndex, identity).GetLocation();

				extent.X = FMath::Max(extent.X, FMath::Abs(boneOffset.X) + (assignment.Width * 0.5f));
				extent.Y = FMath::Max(extent.Y, FMath::Abs(boneOffset.Y) + (assignment.Width * 0.5f));
			}

			if (Antigravity == false)
			{
				extent.Z = FMath::Max(extent.Z, assignment.Radius);
			}
		}

		extent += FVector(10.0f, 10.0f, 10.0f);

		CameraClipBox = FBox(extent * -1.0f, extent);

		BoundingExtent = extent + FVector(5.0f, 5.0f, 10.0f);
	}
}

#pragma endregion VehiclePhysics

#pragma region VehicleContactSensors

/**
* Get the name of a surface from its type.
***********************************************************************************/

FName ABaseVehicle::GetNameFromSurfaceType(EGameSurface surfaceType)
{
	static FName Asphalt("Asphalt");
	static FName Dirt("Dirt");
	static FName Water("Water");
	static FName Rock("Rock");
	static FName Wood("Wood");
	static FName Metal("Metal");
	static FName Grass("Grass");
	static FName Gravel("Gravel");
	static FName Sand("Sand");
	static FName Snow("Snow");
	static FName Field("Field");
	static FName Default("Default");
	static FName Tractionless("Tractionless");
	static FName Unknown("Unknown");

	switch (surfaceType)
	{
	case EGameSurface::Asphalt:
		return Asphalt;
	case EGameSurface::Dirt:
		return Dirt;
	case EGameSurface::Water:
		return Water;
	case EGameSurface::Wood:
		return Wood;
	case EGameSurface::Rock:
		return Rock;
	case EGameSurface::Metal:
		return Metal;
	case EGameSurface::Grass:
		return Grass;
	case EGameSurface::Gravel:
		return Gravel;
	case EGameSurface::Sand:
		return Sand;
	case EGameSurface::Snow:
		return Snow;
	case EGameSurface::Field:
		return Field;
	case EGameSurface::Default:
		return Default;
	case EGameSurface::Tractionless:
		return Tractionless;
	default:
		return Unknown;
	}
}

/**
* Is the vehicle currently with all wheels (more or less) on the ground?
***********************************************************************************/

bool ABaseVehicle::IsPracticallyGrounded(float distance, bool anyWheel)
{
	if (anyWheel == true)
	{
		for (FVehicleWheel& wheel : Wheels.Wheels)
		{
			if (wheel.IsInContact == true)
			{
				return true;
			}
			else
			{
				if (wheel.GetActiveSensor().HasNearestContactPoint(wheel.Velocity, 0.0f) == true)
				{
					if (wheel.GetActiveSensor().GetNearestContactPointDistanceFromTire() < distance)
					{
						return true;
					}
				}
			}
		}

		return false;
	}
	else
	{
		for (FVehicleWheel& wheel : Wheels.Wheels)
		{
			if (wheel.IsInContact == false)
			{
				if (wheel.GetActiveSensor().HasNearestContactPoint(wheel.Velocity, 0.0f) == true)
				{
					if (wheel.GetActiveSensor().GetNearestContactPointDistanceFromTire() > distance)
					{
						return false;
					}
				}
				else
				{
					return false;
				}
			}
		}

		return true;
	}
}

/**
* Get the direction from the vehicle to the nearest driving surface.
***********************************************************************************/

FVector ABaseVehicle::GetSurfaceDirection()
{
	if (GetNumWheels() > 0)
	{
		// All wheels have the same direction, and this will be pointing towards the
		// nearest surface, even though the direction vector that describes the
		// shortest distance to that surface may be something different.

		return Wheels.Wheels[0].GetActiveSensor().GetDirection();
	}

	return GetUpDirection() * -1.0f;
}

/**
* Get the direction from the vehicle to launch weapons from, often opposing the
* nearest surface direction.
***********************************************************************************/

FVector ABaseVehicle::GetLaunchDirection(bool inContact) const
{
	// All wheels have the same direction, and this will be pointing towards the
	// nearest surface, even though the direction vector that describes the
	// shortest distance to that surface may be something different.

	for (const FVehicleWheel& wheel : Wheels.Wheels)
	{
		if ((inContact == false || wheel.GetActiveSensor().IsInContact() == true) &&
			(wheel.GetActiveSensor().HasNearestContactPoint(wheel.Velocity, 0.0f) == true))
		{
			return wheel.GetActiveSensor().GetDirection() * -1.0f;
		}
	}

	FVector zdirection = GetUpDirection();

	return ((zdirection.Z >= 0.0f) ? zdirection : zdirection * -1.0f);
}

/**
* Get the location of the nearest driving surface to the center of the vehicle.
***********************************************************************************/

FVector ABaseVehicle::GetSurfaceLocation() const
{
	FVector normal = FVector::ZeroVector;
	FVector location = GetCenterLocation();

	if (GetNumWheels() >= 3)
	{
		// This assumes all of the wheels have contacts on their active sensors.

		const FVector& direction = Wheels.Wheels[0].GetActiveSensor().GetDirection();
		const FVector& p0 = Wheels.Wheels[0].GetActiveSensor().GetNearestContactPoint();
		const FVector& p1 = Wheels.Wheels[1].GetActiveSensor().GetNearestContactPoint();
		const FVector& p2 = Wheels.Wheels[2].GetActiveSensor().GetNearestContactPoint();

		// Take the contact locations of 3 of the wheels and take a surface normal
		// away from the plane that the 3 contacts form.

		normal = FVector::CrossProduct(p1 - p0, p2 - p0);

		normal.Normalize();

		// Ensure the plane normal is pointing in the correct direction, towards the
		// center location from the plane's location.

		if (FVector::DotProduct(direction, normal) > 0.0f)
		{
			normal *= -1.0f;
		}

		// Now project the center location onto that imaginary plane and return the result.

		return FVector::PointPlaneProject(location, p0, normal);
	}

	return location;
}

/**
* Get the normal of the nearest driving surface.
***********************************************************************************/

FVector ABaseVehicle::GetSurfaceNormal() const
{
	FVector normal = FVector::ZeroVector;

	if (GetNumWheels() >= 3)
	{
		// This assumes all of the wheels have contacts on their active sensors.

		const FVector& direction = Wheels.Wheels[0].GetActiveSensor().GetDirection();
		const FVector& p0 = Wheels.Wheels[0].GetActiveSensor().GetNearestContactPoint();
		const FVector& p1 = Wheels.Wheels[1].GetActiveSensor().GetNearestContactPoint();
		const FVector& p2 = Wheels.Wheels[2].GetActiveSensor().GetNearestContactPoint();

		// Take the contact locations of 3 of the wheels and take a surface normal
		// away from the plane that the 3 contacts form.

		normal = FVector::CrossProduct(p1 - p0, p2 - p0);

		normal.Normalize();

		// Ensure the normal is pointing in the correct direction, towards the vehicle.

		if (FVector::DotProduct(direction, normal) > 0.0f)
		{
			normal *= -1.0f;
		}
	}

	return normal;
}

/**
* Guess the normal of the nearest driving surface.
***********************************************************************************/

FVector ABaseVehicle::GuessSurfaceNormal() const
{
	FVector normal = FVector::ZeroVector;
	int32 numWheels = GetNumWheels();

	// OK, so sometimes we need to know what the surface normal is of the nearest
	// surface even if we're not in good contact with one. As long as we have 3
	// wheels where the contact sensors have sensed a surface we can do this.

	if (numWheels >= 4)
	{
		// Determine which of the wheels have a surface contact detected.

		TArray<FVector> contacts;

		// #TODO: This will break if you pick 3 wheels on a single axle. This will never
		// happen in GRIP though.

		for (const FVehicleWheel& wheel : Wheels.Wheels)
		{
			if (wheel.GetActiveSensor().HasNearestContactPoint(FVector::ZeroVector, 0.0f) == true)
			{
				contacts.Emplace(wheel.GetActiveSensor().GetNearestContactPoint());

				if (contacts.Num() >= 3)
				{
					normal = FVector::CrossProduct(contacts[1] - contacts[0], contacts[2] - contacts[0]);

					normal.Normalize();

					if (FVector::DotProduct(wheel.GetActiveSensor().GetDirection(), normal) > 0.0f)
					{
						normal *= -1.0f;
					}

					return normal;
				}
			}
		}
	}

	return normal;
}

/**
* Do we have a valid surface contact, optionally over a period of seconds.
***********************************************************************************/

bool ABaseVehicle::IsSurfaceDirectionValid(float contactSeconds)
{
	for (FVehicleWheel& wheel : Wheels.Wheels)
	{
		if (wheel.GetActiveSensor().HasNearestContactPoint(wheel.Velocity, contactSeconds))
		{
			return true;
		}
	}

	return false;
}

/**
* Get the average distance of the wheels from the vehicle to the nearest driving
* surface, 0 for not near any driving surface.
***********************************************************************************/

float ABaseVehicle::GetSurfaceDistance(bool discountFrontWheelsWhenRaised, bool closest)
{
	float sum = 0.0f;
	float averageDistance = 0.0f;
	float minDistance = 0.0f;

	for (FVehicleWheel& wheel : Wheels.Wheels)
	{
		float distance = wheel.GetActiveSensor().GetSurfaceDistance();

		// This hack here is to try to keep the vehicle on the ceiling when doing a charged turbo,
		// nothing more than that really.

		if (wheel.HasRearPlacement() == false &&
			discountFrontWheelsWhenRaised == true &&
			Propulsion.RaiseFrontScale > KINDA_SMALL_NUMBER)
		{
			distance = wheel.Radius;
		}

		if (distance != 0.0f)
		{
			sum += 1.0f;
			averageDistance += distance;

			if (minDistance == 0.0f ||
				minDistance > distance)
			{
				minDistance = distance;
			}
		}
	}

	if (sum != 0.0f)
	{
		averageDistance /= sum;
	}

	if (closest == true)
	{
		return minDistance;
	}
	else
	{
		return averageDistance;
	}
}

/**
* Get the location of the bone for a wheel, in world space.
* Optionally clipped on the Y axis to within the bounds of the collision shape.
***********************************************************************************/

FVector ABaseVehicle::GetWheelBoneLocation(const FVehicleWheel& wheel, const FTransform& transform, bool clipToCollision)
{
	if (clipToCollision == true)
	{
		FVector offset = wheel.BoneOffset;

		if (FMath::Abs(offset.Y) > FMath::Abs(wheel.SuspensionForcesOffset.Y))
		{
			offset.Y = wheel.SuspensionForcesOffset.Y;
		}

		return transform.TransformPosition(offset);
	}
	else
	{
		return transform.TransformPosition(wheel.BoneOffset);
	}
}

/**
* Get the location to apply suspension forces to for a particular wheel in world
* space.
***********************************************************************************/

FVector ABaseVehicle::GetSuspensionForcesLocation(const FVehicleWheel& wheel, const FTransform& transform, float deltaSeconds)
{
	FVector offset = wheel.SuspensionForcesOffset;

	return transform.TransformPosition(offset);
}

/**
* Get how much grip we should apply to a particular contact sensor at this time.
***********************************************************************************/

float ABaseVehicle::GetGripRatio(const FVehicleContactSensor& sensor) const
{
	{
		if (sensor.IsInContact() == true)
		{
			return TireFrictionModel->GripVsSuspensionCompression.GetRichCurve()->Eval(sensor.GetNormalizedCompression());
		}
		else
		{
			return 0.0f;
		}
	}
}

#pragma endregion VehicleContactSensors

#pragma region VehicleBasicForces

/**
* Arrest the vehicle until the game has started.
***********************************************************************************/

void ABaseVehicle::ArrestVehicle()
{
	if (Physics.StaticHold.Active == false &&
		Physics.ContactData.ModeTime > 1.0f &&
		Physics.VelocityData.Speed < 100.0f)
	{
		if (Physics.StaticHold.Active == false)
		{
			Physics.StaticHold.Location = VehicleMesh->GetPhysicsLocation();
			Physics.StaticHold.Rotation = VehicleMesh->GetPhysicsQuaternion();
		}

		Physics.StaticHold.Active = true;
	}

	if (Physics.StaticHold.Active == true)
	{
		if (Wheels.BurnoutForce > 0.0f)
		{
			Physics.StaticHold.Location = VehicleMesh->GetPhysicsLocation();
			Physics.StaticHold.Rotation = VehicleMesh->GetPhysicsQuaternion();
		}
	}
}

/**
* Update the power and gearing, returns true if just shifted up a gear.
***********************************************************************************/

void ABaseVehicle::UpdatePowerAndGearing(float deltaSeconds, const FVector& xdirection, const FVector& zdirection)
{
	if (IsVehicleDestroyed() == false)
	{
		int32 topGear = FMath::Max(VehicleEngineModel->GearPowerRatios.Num(), 7) - 1;

		float speed = GetSpeedKPH();
		float measuredGearPosition = speed / GetGearSpeedRange();
		float acceleration = (AI.Speed.DifferenceFromPerSecond(VehicleClock - 0.2f, VehicleClock, GetSpeedMPS() * 100.0f) / 100.0f);
		bool accelerating = (acceleration > -1.0f && Control.ThrottleInput > 0.25f);
		bool decelerating = (acceleration < -1.0f && Control.ThrottleInput < 0.25f);

		// measuredGearPosition contains the gear and the fraction within that gear.

		int32 gear = FMath::FloorToInt(measuredGearPosition);

		// gear is the integral gear we are currently using.

		Propulsion.CurrentGearPosition = measuredGearPosition - gear;

		// CurrentGearPosition is the fraction of the current gear, 1 being max revs.

		Propulsion.GearTime += deltaSeconds;

		// GearTime is the time spent within the current gear.

		// The amount of overlap to give between gears when accelerating or decelerating.

		float revOverlap = 0.333f;
		float currentGearPosition = Propulsion.CurrentGearPosition;
		bool grounded = IsPracticallyGrounded(100.0f);

		// Don't let gear changes happen too frequently, we don't want that
		// awful high-speed switching between gears that can sometimes occur
		// during hard cornering.

		bool keepGear = (Propulsion.GearTime < 1.0f);

		if (gear >= topGear)
		{
			gear = topGear;
			currentGearPosition = measuredGearPosition - topGear;
		}
		else
		{
			// Determine if we're going up or down the gearbox and then over-rev
			// at the top of a gear if accelerating and under-rev at the bottom of a
			// gear if decelerating. Give time between gear changes so you can see if
			// a change is required (rev high where possible).

			if (accelerating == true)
			{
				keepGear |= (gear == Propulsion.LastGear + 1 && currentGearPosition < revOverlap);
			}

			if (decelerating == true)
			{
				keepGear |= (gear == Propulsion.LastGear - 1 && currentGearPosition > 1.0f - revOverlap);
			}
		}

		if (keepGear == true)
		{
			if (gear > Propulsion.LastGear)
			{
				// We're overrevving.

				currentGearPosition += gear - Propulsion.LastGear;
				currentGearPosition = FMath::Min(currentGearPosition, 1.0f + revOverlap);
			}
			else if (gear < Propulsion.LastGear)
			{
				// We're underrevving.

				currentGearPosition -= Propulsion.LastGear - gear;
				currentGearPosition = FMath::Max(currentGearPosition, -revOverlap);
			}

			gear = Propulsion.LastGear;
		}

		// Calculate the launch boost to boost the overall engine power.

		float launchBoostPower = 1.0f;

		// In low gears, the more away from the flat, the more power we give.
		// The reason being, it's hard to accelerate up a steep hill in low gear.

		float inclineHelp = 0.0f;
		float inclineHelpMax = 0.6f;

		if (Propulsion.PistonEngineThrottle > 0.0f)
		{
			// If propelling forwards.
			// If facing downhill then don't do anything, otherwise give more power the more we're facing uphill.

			inclineHelp = (xdirection.Z < 0.0f) ? 0.0f : (FMath::Min(xdirection.Z, inclineHelpMax) / inclineHelpMax);
		}
		else
		{
			// If propelling backwards.
			// If facing uphill then don't do anything, otherwise give more power the more we're facing downhill.

			inclineHelp = (xdirection.Z > 0.0f) ? 0.0f : (FMath::Min(-xdirection.Z, inclineHelpMax) / inclineHelpMax);
		}

		// Translate the position to a based on a power curve for now.
		// This means low power at beginning of gear and high power at the end.
		// By 4th gear, we are often producing maximum power throughout the gear range.

		float maxJetEnginePower = Propulsion.MaxJetEnginePower * launchBoostPower;
		float gearPower = FMath::Lerp(0.0f, 1.0f, inclineHelp);
		float gearPowerRatio = 1.0f;

		if (VehicleEngineModel->GearPowerRatios.Num() > gear)
		{
			gearPowerRatio = VehicleEngineModel->GearPowerRatios[gear];
#if GRIP_STATIC_ACCELERATION
#endif // GRIP_STATIC_ACCELERATION
		}

		if (gearPowerRatio < 1.0f - KINDA_SMALL_NUMBER)
		{
			gearPowerRatio *= AccelerationCoefficient;
		}

		gearPowerRatio = FMath::Min(1.0f, gearPowerRatio);

		float enginePower = maxJetEnginePower;
		float lowPower = FMath::Lerp(enginePower * gearPowerRatio, enginePower, gearPower);

		Propulsion.CurrentJetEnginePower = FMath::Lerp(lowPower, (IsAirborne() == true) ? Propulsion.MaxJetEnginePowerAirborne : maxJetEnginePower, FMath::Pow(FMath::Max(currentGearPosition, 0.0f), 1.5f));

		float j0 = Propulsion.CurrentJetEnginePower;

		// So now we've got all the engine power calculated, let's manage the gearing simulation.

		bool hasStarted = Propulsion.HasStarted;
		float throttleInput = Control.ThrottleInput;

		if (PlayGameMode != nullptr &&
			PlayGameMode->PastGameSequenceStart() == false)
		{
			hasStarted |= AI.WillRevOnStartLine;
		}

		bool shiftedUp = (Propulsion.LastGear < gear);
		bool shiftedDown = (Propulsion.LastGear > gear);

		// Handle the blueprint effects for gear-shifting.

		if (shiftedUp == true)
		{
			GearUpEngaged();
		}
		else if (shiftedDown == true)
		{
			GearDownEngaged();
		}

		Propulsion.LastGear = gear;

		if (shiftedUp == true)
		{
			// If we're shifting up then added a back-end physics impulse if the conditions are right.

			if (Physics.ContactData.Grounded == true &&
				Physics.ContactData.ModeTime > 0.2f &&
				Wheels.HardCompression == false)
			{
				bool valid = (Control.ThrottleInput >= 0.0f) ? Wheels.RearAxleDown : Wheels.FrontAxleDown;

				if (valid == true)
				{
					bool reversing = FVector::DotProduct(xdirection, GetVelocityOrFacingDirection()) < 0.0f;

					if (reversing == false &&
						Antigravity == false)
					{
						float direction = (Wheels.SoftFlipped == true) ? -1.0f : 1.0f;

						// Although this is clearly physics-related, we're leaving it in the general Tick function
						// as it's an impulse which doesn't need any sub-stepping.

						VehicleMesh->AddImpulseAtLocation(zdirection * -75.0f * direction * Physics.CurrentMass, Wheels.RearAxlePosition);
					}
				}
			}
		}
	}
}

#pragma endregion VehicleBasicForces

#pragma region VehicleControls

/**
* Control the forwards / backwards motion.
* The value will be somewhere between -1 and +1, often at 0 or the extremes.
***********************************************************************************/

void ABaseVehicle::Throttle(float value, bool bot)
{
	if (bot == AI.BotDriver)
	{
		bool paused = false;

		if (PlayGameMode != nullptr)
		{
			paused = PlayGameMode->GamePaused == true && AI.BotDriver == false;
		}

		if (paused == false)
		{
			float thrustForce = Control.ThrottleInput;

			Control.RawThrottleInput = FMath::Clamp(value, -1.0f, 1.0f);
			Control.ThrottleInput = Control.RawThrottleInput;

			if (Control.ThrottleInput != 0.0f)
			{
				Propulsion.HasStarted = true;
			}

			if (thrustForce == 0.0f &&
				Control.ThrottleInput > 0.0f)
			{
				ThrustEngaged();

				Control.DecideWheelSpin = true;
			}
			else if (Control.ThrottleInput == 0.0f &&
				thrustForce > 0.0f)
			{
				ThrustDisengaged();
			}

			Control.ThrottleInput = CalculateAssistedThrottleInput();
		}
	}
}

/**
* Control the left / right motion.
* The value will be somewhere between -1 and +1.
***********************************************************************************/

void ABaseVehicle::Steering(float value, bool analog, bool bot)
{
	if (bot == AI.BotDriver)
	{
		bool paused = false;

		if (PlayGameMode != nullptr)
		{
			paused = PlayGameMode->GamePaused == true && AI.BotDriver == false;
		}

		if (bot == false)
		{
			if (GameState->IsTrackMirrored() == true)
			{
				value *= -1.0f;
			}
		}

		if (paused == false)
		{
			value = FMath::Clamp(value, -1.0f, 1.0f);

			if (AI.BotDriver == false &&
				GameState->InputControllerOptions.IsValidIndex(LocalPlayerIndex) == true)
			{
				FInputControllerOptions& input = GameState->InputControllerOptions[LocalPlayerIndex];

				if (FMath::Abs(value) < input.AnalogDeadZone)
				{
					value = 0.0f;
				}

				// Make the sensitivity less responsive at lower levels in the new engine because - because players complaining.

				value = FMathEx::NegativePow(value, 1.0f + ((1.0f - input.SteeringSensitivity) * 4.0f));
			}

			if (analog == true)
			{
				Control.SteeringInputAnalog = value;

				if (bot == true ||
					value != 0.0f)
				{
					Control.SteeringAnalog = true;
				}
			}
			else
			{
				Control.SteeringInputDigital = value;

				if (value != 0.0f)
				{
					Control.SteeringAnalog = false;
				}
			}
		}
	}
}

/**
* Engage the brake.
***********************************************************************************/

void ABaseVehicle::HandbrakePressed(bool bot)
{
	if (bot == AI.BotDriver)
	{
		if (Control.BrakeInput < 0.1f)
		{
			// Determine the braking bias only when the brake is off, and maintain
			// that bias for the duration of the braking action.

			Physics.BrakingSteeringBias = FMathEx::UnitSign(Physics.SteeringBias);
		}

		if (Control.BrakeInput != 1.0f)
		{
			Control.BrakeInput = 1.0f;
			Control.HandbrakePressed = GetRealTimeClock();
		}
	}
}

/**
* Release the brake.
***********************************************************************************/

void ABaseVehicle::HandbrakeReleased(bool bot)
{
	if (bot == AI.BotDriver)
	{
		if (Control.BrakeInput != 0.0f)
		{
			Control.BrakeInput = 0.0f;

			if (RaceState.RaceTime == 0.0f)
			{
				Control.BrakePosition = Control.BrakeInput;
			}
		}
	}
}

/**
* Handle the use of automatic braking to assist the driver.
***********************************************************************************/

float ABaseVehicle::AutoBrakePosition(const FVector& xdirection) const
{
	float speed = GetSpeedKPH();

	if (speed > 5.0f)
	{
		FVector direction = GetVelocityOrFacingDirection();
		float dotProduct = FVector::DotProduct(direction, xdirection);

		// If we're throttling forwards but are actually currently reversing, or we're throttling
		// backwards but actually going forwards, then apply the brake to make the transition to
		// the intended direction of travel pass more quickly.

		if ((Control.ThrottleInput > 0.0f && dotProduct < -0.5f) ||
			(Control.ThrottleInput < 0.0f && dotProduct > +0.5f))
		{
			float ratio = 1.0f - FMathEx::GetRatio(speed, 75.0f, 150.0f);

			return FMath::Max(Control.BrakePosition, ratio);
		}
	}

	return Control.BrakePosition;
}

/**
* Calculate the assisted throttle input for a player.
***********************************************************************************/

float ABaseVehicle::CalculateAssistedThrottleInput()
{
	float finalThrottle = Control.RawThrottleInput;

	return finalThrottle;
}

/**
* Interpolate the control inputs to give smooth changes to digital inputs.
***********************************************************************************/

void ABaseVehicle::InterpolateControlInputs(float deltaSeconds)
{
	float steeringInput = Control.SteeringInputAnalog;
	float steeringInputSpeed = 8.0f;

	if (AI.BotDriver == false)
	{
		// Decide which direction to pitch the vehicle in when using air control.

		if (Control.AirbornePitchInput == 0.0f &&
			Control.AirborneControlActive == true)
		{
			Control.AirborneControlTimer += deltaSeconds;

			if (Control.AirborneControlTimer > 1.0f)
			{
				FMinimalViewInfo viewInfo;

				Camera->GetCameraViewNoPostProcessing(0.0f, viewInfo);

				FVector cameraUp = viewInfo.Rotation.Quaternion().GetUpVector();
				FVector vehicleUp = GetActorRotation().Quaternion().GetUpVector();

				Control.AirborneControlScale = ((FVector::DotProduct(vehicleUp, cameraUp) < 0.0f) ? -1.0f : 1.0f);
			}
		}
		else
		{
			Control.AirborneControlTimer = 0.0f;
		}

		if (Control.SteeringAnalog == true)
		{
			if (LocalPlayerIndex >= 0 &&
				LocalPlayerIndex < GameState->InputControllerOptions.Num())
			{
				FInputControllerOptions& input = GameState->InputControllerOptions[LocalPlayerIndex];

				steeringInputSpeed = 4.0f + (input.AnalogSteeringSpeed * 4.0f);
			}
		}
		else
		{
			steeringInput = Control.SteeringInputDigital;

			if (LocalPlayerIndex >= 0 &&
				LocalPlayerIndex < GameState->InputControllerOptions.Num())
			{
				FInputControllerOptions& input = GameState->InputControllerOptions[LocalPlayerIndex];

				steeringInputSpeed = 4.0f + (input.DigitalSteeringSpeed * 4.0f);
			}
		}

		Control.ThrottleInput = CalculateAssistedThrottleInput();
	}

	// Interpolate the steering and brake positions.

	Control.SteeringPosition = FMathEx::GravitateToTarget(Control.SteeringPosition, steeringInput, deltaSeconds * steeringInputSpeed);
	Control.BrakePosition = FMathEx::GravitateToTarget(Control.BrakePosition, Control.BrakeInput, deltaSeconds * BrakingInputSpeed);

	Control.AirborneRollInput = steeringInput;

	Control.AirborneRollPosition = FMathEx::GravitateToTarget(Control.AirborneRollPosition, Control.AirborneRollInput, deltaSeconds * steeringInputSpeed);
	Control.AirbornePitchPosition = FMathEx::GravitateToTarget(Control.AirbornePitchPosition, Control.AirbornePitchInput, deltaSeconds * steeringInputSpeed);

	if (Physics.ContactData.Airborne == true)
	{
		if (FMath::Abs(Control.ThrottleInput) < 0.25f)
		{
			Propulsion.ThrottleOffWhileAirborne = true;
		}
	}
	else
	{
		Propulsion.ThrottleOffWhileAirborne = false;
	}

	if (PlayGameMode != nullptr)
	{
		if (PlayGameMode->PastGameSequenceStart() == false)
		{
			Control.BrakePosition = 1.0f;
		}

		Control.ThrottleList.AddValue(GameMode->GetRealTimeClock(), Control.ThrottleInput);
	}
}

/**
* Update the steering of the wheels.
***********************************************************************************/

void ABaseVehicle::UpdateSteering(float deltaSeconds, const FVector& xdirection, const FVector& ydirection, const FQuat& quaternion)
{
	// Manage the steering control.

	float speed = GetSpeedKPH();
	float rfb = SteeringModel->FrontSteeringVsSpeed.GetRichCurve()->Eval(speed);
	float rbb = SteeringModel->BackSteeringVsSpeed.GetRichCurve()->Eval(speed);

	float rf = rfb;
	float rb = rbb;

	rf = FMath::Max(rf, 0.001f);
	rb = FMath::Max(rb, 0.001f);

	float steeringPosition = Control.SteeringPosition;

	float mfb = SteeringModel->FrontWheelsMaxSteeringAngle;
	float mbb = SteeringModel->BackWheelsMaxSteeringAngle;

	float mf = mfb;
	float mb = mbb;

	Wheels.BackSteeringAngle = steeringPosition * mb * rb;
	Wheels.FrontSteeringAngle = -steeringPosition * mf * rf;

	if (Wheels.FlipTimer > 0.0f)
	{
		Wheels.BackSteeringAngle = FMath::Lerp(Wheels.BackSteeringAngle, Wheels.BackSteeringAngle * -1.0f, Wheels.FlipTimer);
		Wheels.FrontSteeringAngle = FMath::Lerp(Wheels.FrontSteeringAngle, Wheels.FrontSteeringAngle * -1.0f, Wheels.FlipTimer);
	}

	if (Wheels.SoftFlipped == false)
	{
		Wheels.BackSteeringAngle *= -1.0f;
		Wheels.FrontSteeringAngle *= -1.0f;
	}

	float rf1 = SteeringModel->FrontSteeringVsSpeed.GetRichCurve()->Eval(0);
	float rb1 = SteeringModel->BackSteeringVsSpeed.GetRichCurve()->Eval(0);

	Wheels.FrontVisualSteeringAngle = Wheels.FrontSteeringAngle;
	Wheels.BackVisualSteeringAngle = Wheels.BackSteeringAngle;

	if (rf1 > 0.0f)
	{
		Wheels.FrontVisualSteeringAngle = FMath::Lerp(Wheels.FrontSteeringAngle, Wheels.FrontSteeringAngle * (rf1 / rf), SteeringModel->FrontVisualUnderSteerRatio);
	}

	if (rb1 > 0.0f)
	{
		Wheels.BackVisualSteeringAngle = FMath::Lerp(Wheels.BackSteeringAngle, Wheels.BackSteeringAngle * (rb1 / rb), SteeringModel->BackVisualUnderSteerRatio);
	}

	for (FVehicleWheel& wheel : Wheels.Wheels)
	{
		FRotator steering = FRotator(0.0f, ((wheel.HasRearPlacement() == true) ? Wheels.BackSteeringAngle : Wheels.FrontSteeringAngle), 0.0f);
		float steeringScale = FMathEx::GetRatio(GetSpeedKPH() * FMath::Abs(FVector::DotProduct(GetDirection(), GetVelocityDirection())), 10.0f, 100.0f);

		wheel.SetSteeringTransform(quaternion, steering, steering * steeringScale);
	}
}

/**
* Handle the pitch control for airborne control.
***********************************************************************************/

void ABaseVehicle::PitchControl(float value)
{
	if (AI.BotDriver == false &&
		GameState->InputControllerOptions.IsValidIndex(LocalPlayerIndex) == true)
	{
		FInputControllerOptions& input = GameState->InputControllerOptions[LocalPlayerIndex];

		if (FMath::Abs(value) < input.AnalogDeadZone)
		{
			value = 0.0f;
		}
	}

	Control.AirbornePitchInput = value;
}

#pragma endregion VehicleControls

#pragma region VehicleAnimation

/**
* Update the animated bones.
***********************************************************************************/

void ABaseVehicle::UpdateAnimatedBones(float deltaSeconds, const FVector& xdirection, const FVector& ydirection)
{
	float shiftVertical = 0.0f;

	for (int32 wheelIndex = 0; wheelIndex < GetNumWheels(); wheelIndex++)
	{
		FVehicleWheel& wheel = Wheels.Wheels[wheelIndex];

		if (Antigravity == false)
		{
			// Setup the wheel rotations for rendering with.

			WheelRotations[wheelIndex].Yaw = GetVisualSteeringAngle(wheel);

			// We're rolling the wheel so just add in the rotations for this frame.

			WheelRotations[wheelIndex].Pitch += wheel.RPS * deltaSeconds * 360.0f;
			WheelRotations[wheelIndex].Pitch = FMath::Fmod(WheelRotations[wheelIndex].Pitch, 3600.0f * FMathEx::UnitSign(WheelRotations[wheelIndex].Pitch));
		}

		// Setup the offset of the wheel to be rendered with.

		WheelOffsets[wheelIndex].Z = wheel.GetActiveSensor().GetExtension();

		float travel = MaximumWheelTravel;

		if (WheelOffsets[wheelIndex].Z > travel)
		{
			shiftVertical += WheelOffsets[wheelIndex].Z - travel;
		}
		else if (WheelOffsets[wheelIndex].Z < -travel)
		{
			shiftVertical += WheelOffsets[wheelIndex].Z + travel;
		}

		if (Wheels.FlipTimer > 0.0f)
		{
			FVehicleContactSensor& sensor = wheel.Sensors[wheel.SensorIndex ^ 1];

			WheelOffsets[wheelIndex].Z = FMath::Lerp(WheelOffsets[wheelIndex].Z, sensor.GetExtension(), Wheels.FlipTimer);
		}
	}

	VehicleOffset.Z = shiftVertical / (float)GetNumWheels();

	// Apply a visual roll to add tilt to the vehicle when cornering and most
	// of the wheels are on the ground.

	UpdateVisualRotation(deltaSeconds, xdirection, ydirection);
}

/**
* Apply a visual roll to add tilt to the vehicle when cornering and most of the
* wheels are on the ground.
***********************************************************************************/

void ABaseVehicle::UpdateVisualRotation(float deltaSeconds, const FVector& xdirection, const FVector& ydirection)
{
	float clock = VehicleClock;
	float torqueRoll = (AI.TorqueRoll * 0.15f) + (FMath::Sin(clock * AI.TorqueRoll * 100.0f) * 0.2f * AI.TorqueRoll);

	if (GetSpeedMPS() > 1.0f &&
		Wheels.NumWheelsInContact > (GetNumWheels() >> 1))
	{
		// First calculate the pitch of the vehicle based on acceleration on the vehicle's X axis.
		// This will make the back-end dip when accelerating and raise when decelerating. This would
		// normally be done through dynamic loading on the suspension in a driving simulator but would
		// result in far too much instability in GRIP. So we provide visual indicators only here.

		float ratio = FMathEx::GetSmoothingRatio(0.9f, deltaSeconds);
		float pitch = FMath::Clamp(FMathEx::CentimetersToMeters(Physics.VelocityData.AccelerationLocalSpace.X) * -0.1f * BrakingLeanScale, -BrakingLeanMaximum, BrakingLeanMaximum);

		if (IsFlipped() == false)
		{
			pitch *= -1.0f;
		}

		VehicleRotation.Pitch = FMath::Lerp(pitch, VehicleRotation.Pitch, ratio);

		// Now calculate the roll angle of the vehicle, based on how hard it's cornering.
		// Use the lateral forces on the tires to gauge where we're trying to push the vehicle towards.
		// We use this TwoFrameLateralForceStrength variable as it is an average of lateral force applied
		// over the last couple of frames, and therefore avoids the innate ping-ponging effect lateral
		// forces have of shifting a vehicle one way and then the next when not cornering sufficient hard.

		float lateralForce = 0.0f;
		float lateralForceSum = 0.0f;

		for (FVehicleWheel& wheel : Wheels.Wheels)
		{
			if (wheel.GetActiveSensor().IsInContact() == true)
			{
				lateralForce += wheel.TwoFrameLateralForceStrength;
				break;
			}
		}

		if (lateralForceSum != 0.0f)
		{
			lateralForce /= lateralForceSum;
		}

		ratio = FMathEx::GetSmoothingRatio(0.95f, deltaSeconds);

		// Note that we have to ignore anything under 50KPH as we get rogue forces in this regime.

		float scale = FMath::Pow(FMathEx::GetRatio(GetSpeedKPH(), 50.0f, 250.0f), 0.5f);

		lateralForce *= scale;

		// Now we have the lateral force computed, convert that into a body roll value.

		float roll = lateralForce * 0.04f;

		roll = (FMath::Abs(roll) < 0.25f) ? 0.0f : roll - 0.25f * FMathEx::UnitSign(roll);
		roll = FMath::Clamp(roll * CorneringLeanScale, -CorneringLeanMaximum, CorneringLeanMaximum);
		roll *= 1.0f - Control.BrakePosition;

		if (IsFlipped() == false)
		{
			roll *= -1.0f;
		}

		VehicleRotation.Roll = (VehicleRotation.Roll * ratio) + (roll * (1.0f - ratio)) + torqueRoll;
		VehiclePitchAccumulator = 0.0f;
		VehiclePitchFrom = VehicleRotation.Pitch;
	}
	else
	{
		// Gently kill pitch and roll when moving real slow.

		float ratio = FMathEx::GetSmoothingRatio(0.95f, deltaSeconds);

		VehiclePitchAccumulator += deltaSeconds * 0.5f;

		VehicleRotation.Roll = (VehicleRotation.Roll * ratio) + torqueRoll;
		VehicleRotation.Pitch = FMath::Lerp(VehiclePitchFrom, 0.0f, FMathEx::EaseInOut(FMath::Min(1.0f, VehiclePitchAccumulator), 3.0f));
	}
}

#pragma endregion VehicleAnimation

#pragma region VehicleSpringArm

/**
* Looking forwards or backwards.
***********************************************************************************/

void ABaseVehicle::LookForwards(float val)
{
	float deadZone = 0.0f;

	if (AI.BotDriver == false &&
		LocalPlayerIndex >= 0 &&
		LocalPlayerIndex < GameState->InputControllerOptions.Num())
	{
		FInputControllerOptions& input = GameState->InputControllerOptions[LocalPlayerIndex];

		deadZone = input.AnalogDeadZone;

		if (input.IgnoreRightStick == true)
		{
			return;
		}
	}

	CameraTarget()->SpringArm->LookForwards(val, deadZone);
}

/**
* Looking left or right.
***********************************************************************************/

void ABaseVehicle::LookSideways(float val)
{
	if (GameState->IsTrackMirrored() == true)
	{
		val *= -1.0f;
	}

	float deadZone = 0.0f;

	if (AI.BotDriver == false &&
		LocalPlayerIndex >= 0 &&
		LocalPlayerIndex < GameState->InputControllerOptions.Num())
	{
		FInputControllerOptions& input = GameState->InputControllerOptions[LocalPlayerIndex];

		deadZone = input.AnalogDeadZone;

		if (input.IgnoreRightStick == true)
		{
			return;
		}
	}

	CameraTarget()->SpringArm->LookSideways(val, deadZone);
}

/**
* Looking left.
***********************************************************************************/

void ABaseVehicle::LeftViewCamera()
{
	if (GameState->IsTrackMirrored() == true)
	{
		CameraTarget()->SpringArm->RightViewCamera(GameState->GeneralOptions.InstantaneousLook);
	}
	else
	{
		CameraTarget()->SpringArm->LeftViewCamera(GameState->GeneralOptions.InstantaneousLook);
	}
}

/**
* Looking right.
***********************************************************************************/

void ABaseVehicle::RightViewCamera()
{
	if (GameState->IsTrackMirrored() == true)
	{
		CameraTarget()->SpringArm->LeftViewCamera(GameState->GeneralOptions.InstantaneousLook);
	}
	else
	{
		CameraTarget()->SpringArm->RightViewCamera(GameState->GeneralOptions.InstantaneousLook);
	}
}

/**
* The angle that the rear-end is currently drifting at.
***********************************************************************************/

float ABaseVehicle::GetSpringArmYaw() const
{
	float yaw = GetDriftRatio();

	yaw = FMathEx::NegativePow(yaw, 0.4f);
	yaw = yaw * Physics.Drifting.RearDriftAngle * SpringArm->DriftYawExtension;

	return yaw;
}

/**
* The roll angle.
***********************************************************************************/

float ABaseVehicle::GetSpringArmRoll() const
{
	// This is pretty much just a bit of extra vehicle lean, it's not the entire rotation of the vehicle.

	return (VehicleRotation.Roll * 0.5f) + (GetDriftRatio() * 6.0f * FMathEx::UnitSign(VehicleRotation.Roll));
}

/**
* Has the vehicle just smashed into something and requires the forward-facing
* crash-camera?
***********************************************************************************/

bool ABaseVehicle::HasSmashedIntoSomething(float maxKPH) const
{
	if (PlayGameMode != nullptr)
	{
		float lastSpeed = AI.Speed.GetLastValue();

		if (lastSpeed < FMathEx::KilometersPerHourToCentimetersPerSecond(maxKPH))
		{
			// We're going slow enough, now see if there was a sharp drop-off in speed to get us here.

			float lastTime = AI.Speed.GetLastTime();
			float hundredKPH = FMathEx::KilometersPerHourToCentimetersPerSecond(100.0f);

			for (int32 i = AI.Speed.GetNumValues() - 1; i >= 0; i--)
			{
				if ((lastTime - AI.Speed[i].Time) < 0.5f)
				{
					if ((AI.Speed[i].Value - lastSpeed) > hundredKPH)
					{
						return true;
					}
				}
				else
				{
					break;
				}
			}
		}
	}

	return false;
}

/**
* Update the materials used to render the vehicle based on cockpit-camera state.
***********************************************************************************/

void ABaseVehicle::UpdateCockpitMaterials()
{
	bool isCockpitView = IsCockpitView();

	if (UsingCockpitMaterial != isCockpitView)
	{
		UsingCockpitMaterial = isCockpitView;

		if (isCockpitView == false &&
			BaseMaterials.Num() > 0)
		{
			int32 materialIndex = 0;
			UObject* lastObject = nullptr;

			for (FMeshMaterialOverride& component : BaseMaterials)
			{
				if (lastObject != component.Component)
				{
					materialIndex = 0;
					lastObject = component.Component;
				}

				component.Component->SetMaterial(materialIndex++, component.Material);
			}
		}
		else if (isCockpitView == true)
		{
			if (OurGhostMaterial == nullptr)
			{
				OurGhostMaterial = UMaterialInstanceDynamic::Create(CockpitGhostMaterial, this);
			}

			OurGhostMaterial->SetScalarParameterValue("CentreViewSize", 8.0f);

			if (BaseMaterials.Num() == 0)
			{
				for (int32 j = 0; j < VehicleMesh->GetNumMaterials(); j++)
				{
					BaseMaterials.Emplace(FMeshMaterialOverride(VehicleMesh, VehicleMesh->GetMaterial(j)));
				}

				for (int32 i = 0; i < VehicleMesh->GetNumChildrenComponents(); i++)
				{
					USceneComponent* child = VehicleMesh->GetChildComponent(i);
					UStaticMeshComponent* staticMesh = Cast<UStaticMeshComponent>(child);
					UChildActorComponent* childActor = Cast<UChildActorComponent>(child);

					if (staticMesh != nullptr)
					{
						for (int32 j = 0; j < staticMesh->GetNumMaterials(); j++)
						{
							BaseMaterials.Emplace(FMeshMaterialOverride(staticMesh, staticMesh->GetMaterial(j)));
						}
					}
					else if (childActor != nullptr)
					{
						ACanard* canard = Cast<ACanard>(childActor->GetChildActor());

						if (canard != nullptr)
						{
							for (int32 j = 0; j < canard->CanardMesh->GetNumMaterials(); j++)
							{
								BaseMaterials.Emplace(FMeshMaterialOverride(canard->CanardMesh, canard->CanardMesh->GetMaterial(j)));
							}
						}
					}
				}
			}

			int32 materialIndex = 0;
			UObject* lastObject = nullptr;

			for (FMeshMaterialOverride& component : BaseMaterials)
			{
				if (lastObject != component.Component)
				{
					materialIndex = 0;
					lastObject = component.Component;
				}

				component.Component->SetMaterial(materialIndex++, OurGhostMaterial);
			}
		}
	}

	if (isCockpitView == true)
	{
		OurGhostMaterial->SetScalarParameterValue("CockpitOpacity", GameState->GraphicsOptions.CockpitVehicleVisibility);
	}
}

#pragma endregion VehicleSpringArm

#pragma region VehicleHUD

/**
* Shake the HUD, following an explosion or something.
***********************************************************************************/

void ABaseVehicle::ShakeHUD(float strength)
{
	if (PlayGameMode != nullptr &&
		PlayGameMode->PastGameSequenceStart() == true)
	{
		float shakeStrength = FMath::Sqrt(FMath::Min(strength, 1.0f));

		ShakeController(shakeStrength, FMath::Max(0.1f, shakeStrength * 0.5f), true, false, true, false, EDynamicForceFeedbackAction::Start);

		if (strength > 0.2f)
		{
			float thisMagnitude = HUD.ShakeMagnitude * (HUD.ShakeTimer / HUD.ShakeTime);

			if (thisMagnitude < strength)
			{
				HUD.ShakeTime = 4.0f;
				HUD.ShakeTimer = HUD.ShakeTime;
				HUD.ShakeMagnitude = FMath::Max(thisMagnitude, strength);
			}
		}
	}
}

/**
* Play a 1D client sound.
***********************************************************************************/

void ABaseVehicle::ClientPlaySound(USoundBase* Sound, float VolumeMultiplier, float PitchMultiplier) const
{
	if (IsHumanPlayer() == true &&
		HasAIDriver() == false)
	{
		UGameplayStatics::PlaySound2D(this, Sound, VolumeMultiplier, PitchMultiplier);
	}
}

/**
* Play the denied sound when a player tries to do something that they cannot.
***********************************************************************************/

void ABaseVehicle::PlayDeniedSound()
{
	if (IsHumanPlayer() == true &&
		IsCinematicCameraActive() == false)
	{
		ClientPlaySound(HUD.PickupNotChargeableSound);
	}
}

/**
* Get the speed of the vehicle, in kilometers / miles per hour.
***********************************************************************************/

FString ABaseVehicle::GetFormattedSpeedKPH(int32 index) const
{
	if (GameState->TransientGameState.ShowFPS == true &&
		GameState->GeneralOptions.SpeedUnit != ESpeedDisplayUnit::MACH)
	{
		return FString::Printf(TEXT("%03d"), FMath::RoundToInt(1.0f / PlayGameMode->FrameTimes.GetScaledMeanValue()));
	}
	else
	{
		float speed = GetSpeedKPH(true);

		switch (GameState->GeneralOptions.SpeedUnit)
		{
		case ESpeedDisplayUnit::MPH:
			return FString::Printf(TEXT("%03d"), FMath::FloorToInt(speed * 0.621371f));
		case ESpeedDisplayUnit::KPH:
			return FString::Printf(TEXT("%03d"), FMath::FloorToInt(speed));
		default:
			if (index == 0)
			{
				return FString::Printf(TEXT("%01d"), FMath::FloorToInt(speed * 0.000809848f));
			}
			else
			{
				return FString::Printf(TEXT("%02d"), FMath::FloorToInt(FMath::Frac(speed * 0.000809848f) * 100.0f));
			}
		}
	}
}

/**
* Get a formatted time for racing.
***********************************************************************************/

FString ABaseVehicle::GetFormattedTime(float seconds)
{
	float minutes = FMath::FloorToFloat(seconds / 60.0f);

	seconds -= minutes * 60.0f;

	float thousands = FMath::Frac(seconds) * 1000.0f;

	return FString::Printf(TEXT("%02d:%02d.%03d"), FMath::FloorToInt(minutes), FMath::FloorToInt(seconds), FMath::FloorToInt(thousands));
}

/**
* Is the vehicle going the wrong way around the track?
***********************************************************************************/

bool ABaseVehicle::IsGoingTheWrongWay() const
{
	if (GameState->IsGameModeRace() == true &&
		PlayGameMode->PastGameSequenceStart() == true &&
		PlayGameMode->GameHasEnded() == false)
	{
		if (GetSpeedKPH() > 100.0f &&
			HUD.WrongWayTimer > 2.0f)
		{
			return true;
		}
	}

	return false;
}

/**
* Show a status message.
***********************************************************************************/

void ABaseVehicle::ShowStatusMessage(const FStatusMessage& message, bool queue, bool inChatIfPossible) const
{
	if (HUDWidget != nullptr)
	{
		if (inChatIfPossible == true &&
			HUDWidget->GetEventPlayingVisibility() != ESlateVisibility::Collapsed)
		{
			if (PlayGameMode != nullptr)
			{
				FGameEvent event;

				event.LaunchVehicleIndex = -1;
				event.EventType = EGameEventType::ChatMessage;
				event.ExtraInformation = message.Message.ToString();

				PlayGameMode->AddGameEvent(event);
			}
		}
		else
		{
			HUDWidget->ShowStatusMessage(message, queue);
		}
	}
}

/**
* Get the alpha value of the wrong way indicator.
***********************************************************************************/

float ABaseVehicle::GetWrongWayAlpha()
{
	if (IsGoingTheWrongWay() == true)
	{
		return (FMath::Fmod(HUD.WrongWayTimer, 1.0f) * 0.5f < 0.25f) ? 1.0f : 0.0f;
	}

	return 0.0f;
}

#pragma endregion VehicleHUD

#pragma region ClocksAndTime

/**
* Reset the timer used for controlling attack frequency.
***********************************************************************************/

void ABaseVehicle::ResetAttackTimer()
{
	float attackDelay = PlayGameMode->GetDifficultyCharacteristics().PickupUseCharacteristics.Race.MaxHumanAttackFrequency;

	attackDelay = FMath::Max(attackDelay, FMath::Lerp(attackDelay, 50.0f, FMath::Min(1.0f, PlayGameMode->LastLapRatio * 1.5f)));

	AttackAfter = VehicleClock + FMath::FRandRange(attackDelay, attackDelay * 1.25f);
}

#pragma endregion ClocksAndTime

#pragma region Miscellaneous

/**
* Set whether the vehicle should use an AI driver or not.
***********************************************************************************/

void ABaseVehicle::SetAIDriver(bool aiDriver, bool setVehicle, bool setInputMappings)
{
	if (AI.BotDriver != aiDriver)
	{
		AI.BotDriver = aiDriver;

		if (AI.BotDriver == true)
		{
		}
		else
		{
			HandbrakeReleased(false);
		}
	}

	if (setVehicle == true)
	{
		AI.BotVehicle = AI.BotDriver;
	}

	if (setInputMappings == true)
	{
		GameMode->SetInputOptions(Cast<APlayerController>(GetController()));
	}
}

/**
* Add points to the player's total if the player's game hasn't ended.
***********************************************************************************/

bool ABaseVehicle::AddPoints(int32 numPoints, bool visualize, ABaseVehicle* fromVehicle, const FVector& worldLocation)
{
	if ((numPoints > 0) &&
		(IsVehicleDestroyed() == false))
	{
		if (RaceState.AddPoints(numPoints) == true)
		{
			return true;
		}
	}

	return false;
}

/**
* Get the progress through the game event, from 0 to 1.
***********************************************************************************/

float ABaseVehicle::GetEventProgress()
{
	if (GameState->IsGameModeLapBased() == true)
	{
		RaceState.EventProgress = FMath::Min(RaceState.RaceDistance / (PlayGameMode->MasterRacingSplineLength * GameState->GeneralOptions.NumberOfLaps), 1.0f);
	}
	else
	{
		RaceState.EventProgress = 0.0f;
	}

	return RaceState.EventProgress;
}

/**
* Cycle through the camera points on the vehicle.
***********************************************************************************/

void ABaseVehicle::CycleCameraPoint()
{
}

/**
* Should the vehicle turn left to head in the correct direction?
***********************************************************************************/

bool ABaseVehicle::ShouldTurnLeft() const
{
	return false;
}

/**
* Should the vehicle turn right to head in the correct direction?
***********************************************************************************/

bool ABaseVehicle::ShouldTurnRight() const
{
	return false;
}

/**
* Does this vehicle belong to a human player?
***********************************************************************************/

int32 ABaseVehicle::DetermineLocalPlayerIndex()
{
	ControllerID = INDEX_NONE;
	LocalPlayerIndex = INDEX_NONE;

	if (IsHumanPlayer() == true)
	{
		APlayerController* controller = Cast<APlayerController>(GetController());

		if (controller != nullptr)
		{
			int32 index = 0;

			// #TODO: Check this indexing method, in SP and SS.

			for (TActorIterator<AController> actorItr(GetWorld()); actorItr; ++actorItr)
			{
				if (*actorItr == controller)
				{
					LocalPlayerIndex = index;
					break;
				}

				index++;
			}

			ControllerID = controller->GetLocalPlayer()->GetControllerId();
		}
	}

	return LocalPlayerIndex;
}

/**
* Disqualify this player from the game event.
***********************************************************************************/

void ABaseVehicle::Disqualify()
{
	if (PlayGameMode != nullptr &&
		RaceState.PlayerCompletionState < EPlayerCompletionState::Complete)
	{
		RaceState.GameFinishedAt = PlayGameMode->GetRealTimeClock();
	}

	RaceState.PlayerCompletionState = EPlayerCompletionState::Disqualified;
	RaceState.RaceRank = -1;
	RaceState.RacePosition = -1;
}

/**
* Perform some initialization on the vehicle post spawn.
***********************************************************************************/

void ABaseVehicle::PostSpawn(int32 vehicleIndex, bool isLocalPlayer, bool bot)
{
	// NOTE: You cannot rely on PreInitializeComponents, PostInitializeComponents or
	// anything else having been called before this function executes. It will have
	// for automatically created pawns like the local players, but for bots for
	// example, this will be the first function called in that execution chain.

	UE_LOG(GripLog, Log, TEXT("ABaseVehicle::PostSpawn"));

	PostSpawnStarted = true;

	World = GetWorld();
	GameMode = ABaseGameMode::Get(this);
	PlayGameMode = APlayGameMode::Get(this);
	GameState = UGlobalGameState::GetGlobalGameState(this);

	VehicleIndex = vehicleIndex;

	AI.BotDriver = AI.BotVehicle = bot;
	AI.DifficultyLevel = GameState->GeneralOptions.DifficultyLevel;

	if (isLocalPlayer == true)
	{
		DetermineLocalPlayerIndex();
	}

	if (PlayGameMode != nullptr)
	{
		PlayGameMode->DetermineVehicles();
	}

	if (HasActorBegunPlay() == true)
	{
		CompletePostSpawn();
	}
}

/**
* Complete the post spawn sequence.
***********************************************************************************/

void ABaseVehicle::CompletePostSpawn()
{
	if (PostSpawnStarted == true &&
		PostSpawnComplete == false)
	{
		UE_LOG(GripLog, Log, TEXT("ABaseVehicle::CompletePostSpawn"));

		PostSpawnComplete = true;

		if (PlayGameMode != nullptr)
		{
			PlayGameMode->AddAvoidable(this);

			TWeakObjectPtr<UPursuitSplineComponent> mainSpline = PlayGameMode->MasterRacingSpline;

			if (mainSpline != nullptr)
			{
				if (PlayGameMode->MasterRacingSplineStartDistance != 0.0f &&
					PlayGameMode->UnknownPlayerStart == false)
				{
					ensureAlwaysMsgf(RaceState.DistanceAlongMasterRacingSpline < PlayGameMode->MasterRacingSplineStartDistance, TEXT("Player in front of starting line (%f %f)"), RaceState.DistanceAlongMasterRacingSpline, PlayGameMode->MasterRacingSplineStartDistance);
				}
			}
		}
	}
}

/**
* Get the target heading for the vehicle, roughly what direction it should be
* heading in for this part of the track.
***********************************************************************************/

FVector ABaseVehicle::GetTargetHeading() const
{
	{
		return GetFacingDirection();
	}
}

/**
* Get the target vehicle for the camera.
***********************************************************************************/

ABaseVehicle* ABaseVehicle::CameraTarget()
{
	ABaseVehicle* result = this;

	return result;
}

/**
* Get the name of the player, optionally shortened or full.
***********************************************************************************/

const FString& ABaseVehicle::GetPlayerName(bool shortened, bool full)
{
	if (PlayerNamesValid == false)
	{
		// If we've gotten to here, we're either in offline mode or for some reason getting the
		// online name failed and we've not got it cached yet.

		if (IsHumanPlayer() == true &&
			LocalPlayerIndex == 0)
		{
			PlayerName = ABaseGameMode::GetPlayerName(GetPlayerState(), 1, true);
		}
		else
		{
			int32 playerNumber = (LocalPlayerIndex != INDEX_NONE) ? LocalPlayerIndex + 1 : VehicleIndex + 1;

			PlayerName = ABaseGameMode::GetPlayerName(GetPlayerState(), playerNumber, true, true);
		}

		ShortPlayerName = ABaseGameMode::ShortenString(PlayerName, 20);

		PlayerNamesValid = true;
	}

	return (shortened == true) ? ShortPlayerName : PlayerName;
}

/**
* Spawn an appropriately scaled particle system on the vehicle.
***********************************************************************************/

UParticleSystemComponent* ABaseVehicle::SpawnParticleSystem(UParticleSystem* emitterTemplate, FName attachPointName, FVector location, FRotator rotation, EAttachLocation::Type locationType, float scale, bool autoDestroy)
{
	UParticleSystemComponent* component = nullptr;

	if (emitterTemplate != nullptr)
	{
		component = NewObject<UParticleSystemComponent>(RootComponent->GetOwner());

		component->bAutoDestroy = autoDestroy;
		component->bAllowAnyoneToDestroyMe = true;
		component->SecondsBeforeInactive = 0.0f;
		component->bAutoActivate = false;
		component->SetTemplate(emitterTemplate);
		component->bOverrideLODMethod = false;

		GRIP_ATTACH(component, RootComponent, attachPointName);

		if (locationType == EAttachLocation::KeepWorldPosition)
		{
			component->SetWorldLocationAndRotation(location, rotation);
		}
		else
		{
			component->SetRelativeLocationAndRotation(location, rotation);
		}

		if (scale < KINDA_SMALL_NUMBER)
		{
			scale = 1.0f;
		}

		component->SetRelativeScale3D(AttachedEffectsScale * scale);
		component->RegisterComponent();
		component->ActivateSystem(true);
	}

	return component;
}

/**
* Shakes the user GamePad, according to strength and duration.
***********************************************************************************/

void ABaseVehicle::ShakeController(float strength, float duration, bool smallLeft, bool smallRight, bool largeLeft, bool largeRight, TEnumAsByte<EDynamicForceFeedbackAction::Type> action)
{
	if (AI.BotDriver == false &&
		IsVehicleDestroyed() == false)
	{
		if (PlayGameMode != nullptr &&
			PlayGameMode->PastGameSequenceStart() == true)
		{
			if (LocalPlayerIndex >= 0 &&
				LocalPlayerIndex < GameState->InputControllerOptions.Num())
			{
				if (GameState->InputControllerOptions[LocalPlayerIndex].UseForceFeedback == true)
				{
					APlayerController* controller = Cast<APlayerController>(Controller);

					if (controller != nullptr)
					{
						strength *= GameState->InputControllerOptions[LocalPlayerIndex].ForceFeedbackStrength;

						Control.ForceFeedbackHandle = controller->PlayDynamicForceFeedback(strength, FMath::Clamp(duration, 0.1f, 0.5f), largeLeft, smallLeft, largeRight, smallRight, action, Control.ForceFeedbackHandle);
					}
				}
			}
		}
	}
}

/**
* Shake the camera.
***********************************************************************************/

bool ABaseVehicle::ShakeCamera(float strength)
{
	bool result = false;

	if (ImpactCameraShake != nullptr)
	{
		if (PlayGameMode != nullptr &&
			PlayGameMode->PastGameSequenceStart() == true)
		{
			if (IsHumanPlayer() == true &&
				IsCinematicCameraActive() == false)
			{
				APlayerController* controller = Cast<APlayerController>(Controller);

				if (controller != nullptr &&
					controller->IsLocalController())
				{
					controller->ClientPlayCameraShake(ImpactCameraShake, strength);

					result = true;
				}
			}
		}
	}

	return result;
}

/**
* Begin teleportation.
***********************************************************************************/

void ABaseVehicle::BeginTeleport()
{
}

/**
* Handle the update of the idle locking, ensuring the vehicle stays still at very
* low speed rather than subtly sliding around.
***********************************************************************************/

void ABaseVehicle::UpdateIdleLock()
{
	VehicleMesh->UpdateIdleLock(false);

	if (VehicleMesh->IsIdle() == false)
	{

#pragma region VehicleGrip

		// Determine if the vehicle is idle and lock it in place if it is.

		if (Antigravity == false &&
			IsGrounded() == true &&
			GetSpeedKPH() <= 1.0f &&
			FMath::Abs(Control.ThrottleInput) <= 0.1f &&
			FMath::Abs(FVector::DotProduct(GetLaunchDirection(), FVector(0.0f, 0.0f, 1.0f))) > 0.5f)
		{
			bool idle = true;

			for (FVehicleWheel& wheel : Wheels.Wheels)
			{
				if (wheel.GetActiveSensor().IsAtRest() == false ||
					wheel.GetActiveSensor().IsInContact() == false ||
					wheel.GetActiveSensor().GetHitResult().Component->Mobility != EComponentMobility::Static)
				{
					idle = false;
					break;
				}
			}

			if (idle == true)
			{
				VehicleMesh->IdleAt(GetActorLocation(), GetActorQuat());
			}
		}

#pragma endregion VehicleGrip

	}
	else
	{
		// Come out of idle lock if we've gained any speed on throttle from the player or bot.

		if (GetSpeedKPH() > 1.0f ||
			FMath::Abs(Control.ThrottleInput) > 0.1f)
		{
			VehicleMesh->IdleUnlock();
		}
	}
}

#pragma endregion Miscellaneous

/**
* Construct a canard.
***********************************************************************************/

ACanard::ACanard()
{
	CanardMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("CanardMesh"));

	CanardMesh->SetCollisionEnabled(ECollisionEnabled::Type::NoCollision);
	CanardMesh->SetCollisionProfileName(UCollisionProfile::NoCollision_ProfileName);
	CanardMesh->SetGenerateOverlapEvents(false);
	CanardMesh->Mobility = EComponentMobility::Movable;

	SetRootComponent(CanardMesh);
}

#pragma endregion Vehicle
