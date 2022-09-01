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

	RaceState.Tick(deltaSeconds, PlayGameMode, GameState);

	// If we're now finished playing as a result of that Tick, then hand
	// over to AI control now.

	if (AI.BotDriver == false &&
		RaceState.PlayerCompletionState >= EPlayerCompletionState::Complete)
	{
		SetAIDriver(true);
	}

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

	VehicleClock += deltaSeconds;
	Physics.Drifting.Timer += deltaSeconds;

	if (PlayGameMode != nullptr)
	{
		int32 totalVehicles = PlayGameMode->GetVehicles().Num();

		Clock0p5.Tick(VehicleIndex, totalVehicles);
		Clock0p25.Tick(VehicleIndex, totalVehicles);
		Clock0p1.Tick(VehicleIndex, totalVehicles);
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
