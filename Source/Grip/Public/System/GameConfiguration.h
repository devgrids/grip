/**
*
* Game configuration settings.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Some general macros and switches to be used throughout the game.
*
***********************************************************************************/

#pragma once

#include "engine.h"
#include "logging/logmacros.h"
#include "system/strippingconfiguration.h"
#include "runtime/launch/resources/version.h"

#define GRIP_ENGINE_PHYSICS_MODIFIED 1							// Change this to 0 if you didn't modify the engine source code

#ifndef UE_BUILD_DEBUG
#define UE_BUILD_DEBUG 0
#endif

// Declare some logs to be used in the game.

DECLARE_LOG_CATEGORY_EXTERN(GripLog, Log, All);
DECLARE_LOG_CATEGORY_EXTERN(GripAILog, Log, All);
DECLARE_LOG_CATEGORY_EXTERN(GripTeleportationLog, Log, All);

#define GRIP_ENGINE_EXTENDED_MODIFICATIONS 0					// These are extended engine changes which we've not made in this course to keep things simple
#define GRIP_DEBUG_HOMING_MISSILE 0								// Debug diagnostics for the homing missile
#define GRIP_USE_STEAM 1										// Should this build include Steam integration?
#define GRIP_HAS_ONLINE_SUBSYSTEM 1								// Does this build feature an online subsystem?
#define GRIP_GENERIC_PLAYER_NAME !GRIP_HAS_ONLINE_SUBSYSTEM		// Use generic names for players
#define GRIP_VARIABLE_MASS_AND_INERTIA_TENSOR 1					// Use variable mass and inertia tensor to reduce the vehicle's natural tendency to bounce upon landings
#define GRIP_VEHICLE_SUSPENSION_BOUNCE_MITIGATION 1				// Try to soften the vehicle's natural tendency to bounce upon landings due to suspension response
#define GRIP_VEHICLE_SUSPENSION_BOUNCE_NORMALIZE 1				// Try to normalize the vehicle's suspension bounce upon landings
#define GRIP_VEHICLE_BOUNCE_CONTROL 1							// Directly control hard landings to kill natural bounce and replace it with controlled, forced bounce
#define GRIP_MANAGE_MAX_ANGULAR_VELOCITY 1						// Manage the maximum angular velocity of the vehicle
#define GRIP_ANTI_SKYWARD_LAUNCH 1								// Work to prevent skyward launches due to collisions
#define GRIP_NORMALIZED_WEIGHT_ON_WHEEL 1						// Try to prevent the tire grip from acting asymmetrically
#define GRIP_NORMALIZE_GRIP_ON_LANDING 1						// Normalize the tire grip on landing to avoid asymmetrical forces just for a moment
#define GRIP_STATIC_ACCELERATION 0								// Flatten out the gear acceleration between different engine powers - now unwanted hack
#define GRIP_VEHICLE_AUTO_TUNNEL_STEERING 1						// Avoid the tumble dryer effect when steering in tunnels
#define GRIP_MAX_PLAYERS 10										// The maximum number of players in an event
#define GRIP_MAX_LOCAL_PLAYERS 4								// The maximum number of local players in an event
#define GRIP_STEERING_ACTIVE 0.1f								// The amount of steering that needs to be applied before it's considered active
#define GRIP_STEERING_PURPOSEFUL 0.333f							// The amount of steering that needs to be applied before it's considered purposeful
#define GRIP_UNSPECIFIED_CONTROLLER_INPUT -100.0f				// Indicator for an unspecified controller input
#define GRIP_ANTIGRAVITY_TILT_COMPENSATION 1					// Apply tilt compensation on antigravity vehicles, having them behave more realistically
#define GRIP_NERF_ANTIGRAVITY_BOOST 1							// Nerf the boost for antigravity vehicles to have them be less dominating over the classic vehicles
#define GRIP_ANTIGRAVITY_LAGGY_STEERING 0.333f					// The amount of lag to apply to steering on antigravity vehicles
#define GRIP_SPLINE_MOVEMENT_MULTIPLIER 4.0f					// Multiplier for determining nearest spline distance when a vehicle moves
#define GRIP_DOUBLE_DAMAGE_SECONDS 20							// How long double damage lasts for
#define GRIP_ELIMINATION_SECONDS 30								// The time between vehicle eliminations
#define GRIP_ELIMINATION_WARNING_SECONDS 15						// How long the elimination warning sounds for
#define GRIP_TELEPORT_SPAM_PERIOD 1.0f							// The minimum time between being able to use the vehicle teleport feature
#define GRIP_UNLIMITED_TEXTURE_POOL 1							// Unlimited texture pool size, useful on PC for avoiding texture degradation during the game
#define GRIP_FIX_REVERB_FADE_TIMES 1							// Hack to set the fade times for reverb on audio volumes to zero to avoid audio artifacts, may no longer be required

// Control of some human hominid behaviors.

#define GRIP_HOM_LEADING_SLOWDOWN 1								// Have hominids slow down the more they're leading
#define GRIP_HOM_TRAILING_SPEEDUP 1								// Have hominids speed up the more they're trailing
#define GRIP_HOM_TRAILING_GRIPINESS GRIP_HOM_TRAILING_SPEEDUP	// Have hominids increase cornering grip the more they're trailing

// Control of some AI bot behaviors.

#define GRIP_BOT_SLOPPY_STEERING 1								// Have bots have slowly reacting steering the more they're leading
#define GRIP_BOT_LEADING_SLOWDOWN 1								// Have bots slow down the more they're leading (regardless of whether catchup is switched on by the player)
#define GRIP_BOT_LEADING_SLIPPINESS 1							// Have bots have slippery grip the more they're leading
#define GRIP_BOT_TRAILING_SPEEDUP 1								// Have bots speed up the more they're trailing (only if catchup is switched on by the player)
#define GRIP_BOT_TRAILING_GRIPINESS GRIP_BOT_TRAILING_SPEEDUP	// Have bots increase cornering grip the more they're trailing
#define GRIP_BOT_INTELLIGENT_SPEEDVSGRIP 1						// Have bots use intelligent optimum speed calculation based on their cornering grip

#define GRIP_FIXED_TIMING 0										// Use fixed rather than dynamic engine timing, usually used for physics testing

#if GRIP_FIXED_TIMING
#define GRIP_TIMING_FPS 60.0f									// If fixed timing, then how many frames per second
#define GRIP_PHYSICS_SUBSTEPS 2									// If fixed timing, then how many physics sub-steps per frame
#endif

#define GRIP_CYCLE_SUSPENSION_NONE 0							// No suspension cycling
#define GRIP_CYCLE_SUSPENSION_BY_AXLE 1							// Axle suspension cycling
#define GRIP_CYCLE_SUSPENSION GRIP_CYCLE_SUSPENSION_BY_AXLE		// The type of suspension cycling to be performed

// Some useful macros.

#define GRIP_GAME_MODE_LIST(list, localList) check(this != nullptr); auto& localList = APlayGameMode::Get(this)->list;
#define GRIP_GAME_MODE_LIST_FROM(list, localList, gameMode) auto& localList = gameMode->list;
#define GRIP_GAME_MODE_LIST_FOR(list, localList, worldObject) auto& localList = APlayGameMode::Get(worldObject)->list;
#define GRIP_GAME_MODE_LIST_FOR_FROM(list, localList, gameMode) auto& localList = gameMode->list;
#define GRIP_ADD_TO_GAME_MODE_LIST(list) { check(this != nullptr); auto gameMode = APlayGameMode::Get(this); if (gameMode != nullptr) { auto& modeList = gameMode->list; if (modeList.Num() == 0 || modeList.Contains(this) == false) modeList.Emplace(this); } }
#define GRIP_ADD_TO_GAME_MODE_LIST_FROM(list, gameMode) { check(this != nullptr); auto& modeList = gameMode->list; if (modeList.Num() == 0 || modeList.Contains(this) == false) modeList.Emplace(this); }
#define GRIP_REMOVE_FROM_GAME_MODE_LIST(list) { check(this != nullptr); auto gameMode = APlayGameMode::Get(this); if (gameMode != nullptr) { auto& modeList = gameMode->list; if (modeList.Num() > 0 && modeList.Contains(this) == true) modeList.Remove(this); } }
#define GRIP_REMOVE_FROM_GAME_MODE_LIST_FROM(list, gameMode) { auto& modeList = gameMode->list; if (modeList.Num() > 0 && modeList.Contains(this) == true) modeList.Remove(this); }
#define GRIP_ATTACH(child, parent, socket) child->AttachToComponent(parent, FAttachmentTransformRules::KeepRelativeTransform, socket); child->SetRelativeLocation(FVector::ZeroVector); child->SetRelativeRotation(FRotator::ZeroRotator);
#define GRIP_ATTACH_AT(child, parent, socket, offset) child->AttachToComponent(parent, FAttachmentTransformRules::KeepRelativeTransform, socket); child->SetRelativeLocation(offset); child->SetRelativeRotation(FRotator::ZeroRotator);
#define GRIP_DETACH(child) child->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);
#define GRIP_VEHICLE_EFFECT_ATTACH(child, parent, socket, relative) child->AttachToComponent(parent->VehicleMesh, FAttachmentTransformRules::KeepRelativeTransform, socket); child->SetRelativeLocation(FVector::ZeroVector); child->SetRelativeRotation(FRotator::ZeroRotator); if (relative == false) child->SetRelativeScale3D(parent->AttachedEffectsScale);
#define GRIP_OBJECT_VALID(object) (object != nullptr && object->IsValidLowLevel() == true && object->IsPendingKill() == false)
#define GRIP_POINTER_VALID(pointer) (pointer.IsValid() == true)
#define GRIP_NUM_ELEMENTS(array) (sizeof(array) / sizeof(array[0]))

#define GRIP_UE_VERSION_CHECK(major, minor) (ENGINE_MAJOR_VERSION * 1000) + ENGINE_MINOR_VERSION >= (major * 1000) + minor
