/**
*
* The race state for a player.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* Management of the state of a player driving a vehicle in the game. This normally
* handles the event progress and scoring. Applicable to both humans and bots.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"
#include "system/commontypes.h"
#include "system/timesmoothing.h"

class UGlobalGameState;
class APlayGameMode;
class ABaseVehicle;

/**
* How did the game end for a player?
***********************************************************************************/

enum class EPlayerCompletionState : uint8
{
	// Playing the game as normal
	Incomplete,

	// Currently suspended awaiting respawn
	Suspended,

	// Completed end-of-game requirements successfully
	Complete,

	// Destroyed but still complete
	Abandoned,

	// Destroyed or retired normally
	Disqualified
};

/**
* The race state for a particular player, one owned by each vehicle in the game.
***********************************************************************************/

struct FPlayerRaceState
{
public:

	// Construct a player race state.
	FPlayerRaceState(ABaseVehicle* player = nullptr)
		: PlayerVehicle(player)
	{ HitPoints = MaxHitPoints; }

	// Do the regular update tick.
	void Tick(float deltaSeconds, APlayGameMode* gameMode, UGlobalGameState* gameState);

	// Complete the event for the player.
	void PlayerComplete(bool setCompletionStatus, bool gameComplete, bool estimateRaceTime, EPlayerCompletionState completionState = EPlayerCompletionState::Complete);

	// Is the race state locked now and the result properties in it fixed?
	bool IsAccountingClosed() const
	{ return (PlayerCompletionState >= EPlayerCompletionState::Complete); }

	// Update the checkpoints for this player race state to determine their progress around the track.
	void UpdateCheckpoints(bool ignoreCheckpointSize);

	// Add points to the player's total if the player's game hasn't ended.
	bool AddPoints(int32 numPoints);

	int32 GetDamagePercent() const
	{ return (int32)((1.0f - ((float)HitPoints / (float)MaxHitPoints)) * 100.0f); }

	// The vehicle for the player.
	ABaseVehicle* PlayerVehicle = nullptr;

	// The current lap time.
	float LapTime = 0.0f;

	// The last lap time.
	float LastLapTime = 0.0f;

	// The best lap time.
	float BestLapTime = 0.0f;

	// The current race time.
	float RaceTime = 0.0f;

	// The current lap number.
	int32 LapNumber = -1;
	int32 EternalLapNumber = -1;

	// The maximum lap number reached so far (player can go round backwards and decrease their lap count, yes!).
	int32 MaxLapNumber = -1;

	// The position within the race, starting at 0 meaning in the lead, -1 meaning disqualified or invalid.
	int32 RacePosition = -1;

	// The rank within the race, starting at 0 meaning in the lead.
	int32 RaceRank = -1;

	// The number of checkpoints reached.
	int32 CheckpointsReached = 0;

	// The progressive distance traveled around the track based on the main spline progression, in centimeters.
	float RaceDistance = 0.0f;
	float EternalRaceDistance = 0.0f;

	// The progressive distance traveled around the track for this lap only based on the main spline progression, in centimeters.
	float LapDistance = 0.0f;

	// The ratio of boost to be applied to the vehicle. -1 = max speedup and 1 = max slowdown.
	float BoostCatchupRatio = 0.0f;

	// The catchup ratio of this vehicle compared to the mean vehicle (human or bot) distance. -1 = max speedup and 1 = max slowdown.
	float StockCatchupRatioUnbounded = 0.0f;

	// The ratio of catchup to be applied to the vehicle. -1 = max speedup and 1 = max slowdown.
	float RaceCatchupRatio = 0.0f;

	// The ratio of catchup to be applied to drag of the vehicle. -1 = max speedup and 1 = max slowdown.
	float DragCatchupRatio = 0.0f;

	// The scale to apply to the drag of the vehicle, 1 being no change to normal drag.
	float DragScale = 1.0f;

	// The hit points remaining for the vehicle.
	int32 HitPoints = 0;

	// The maximum assigned hit points for the vehicle.
	int32 MaxHitPoints = 0;

	// The number of hit points dealt.
	int32 HitPointsDealt = 0;

	// The number of hit points received.
	int32 HitPointsReceived = 0;

	// The number of seconds left for double damage.
	float DoubleDamage = 0.0f;

	// Has play completed for this player?
	// i.e. the end-game state has been met or the vehicle has been disqualified (e.g. permanently destroyed).
	EPlayerCompletionState PlayerCompletionState = EPlayerCompletionState::Incomplete;

	// Is the player currently hidden?
	bool PlayerHidden = false;

	// Has the player just completed a lap?
	bool LapCompleted = false;

	// What time into the game did this player finish? (Only set under normal circumstances and not when a game end is prematurely forced).
	float GameFinishedAt = 0.0f;

	// The number of frags by the player.
	int32 NumKills = 0;

	// The number of deaths to the player.
	int32 NumDeaths = 0;

	// The additional number of points for the player. This includes NumEventPoints and ongoing measurements (like air-time) that can be measured in-game and will never decrement.
	// All of this is XP.
	int32 NumInGamePoints = 0;

	// The total number of points for the player. This includes NumInGamePoints but also other points such as ongoing measurements (like air-time) and final stats (like highest maximum speed, but only when the game is complete and they can be calculated).
	// All of this is XP.
	int32 NumTotalPoints = 0;

	// The distance on the master racing spline at this point in time.
	float DistanceAlongMasterRacingSpline = 0.0f;

	// The last distance on the master racing spline at this point in time.
	float LastDistanceAlongMasterRacingSpline = 0.0f;

	// The last known pretty much grounded distance on the master racing spline.
	float GroundedDistanceAlongMasterRacingSpline = 0.0f;

	// The progress value for this vehicle as last sent from a remote player.
	float EventProgress = 0.0f;

private:

	// The last checkpoint index.
	int32 LastCheckpoint = -1;

	// The next checkpoint index.
	int32 NextCheckpoint = -1;
};
