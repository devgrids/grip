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

#include "game/playerracestate.h"
#include "game/globalgamestate.h"
#include "vehicle/flippablevehicle.h"
#include "ui/hudwidget.h"
#include "gamemodes/playgamemode.h"

/**
* Do the regular update tick.
***********************************************************************************/

void FPlayerRaceState::Tick(float deltaSeconds, APlayGameMode* gameMode, UGlobalGameState* gameState)
{
	LapCompleted = false;

	if (gameMode != nullptr)
	{
		if (gameMode->PastGameSequenceStart() == true)
		{
			if (PlayerCompletionState < EPlayerCompletionState::Complete)
			{
				// Update the race time for the player.

				float frameTime = gameMode->GetRealTimeGameClock() - RaceTime;

				// We always use real-time for lap and race times, and don't rely on deltaSeconds.

				LapTime += frameTime;
				RaceTime += frameTime;

				// Do the normal end-of-game detection checks for the player and signal completion if met.

				if (gameState->IsGameModeRace() == true)
				{
					if (gameMode->NoOpponentsLeft() == true &&
						gameMode->GetNumOpponents() > 1)
					{
						PlayerComplete(true, true, false);
					}
				}

				if (gameState->IsGameModeRace() == true)
				{
					// We're in a race-type scenario so let's do that particular management.

#pragma region GameModeElimination

					float eliminationRatio = 0.0f;
					FVehicleElimination& elimination = PlayerVehicle->GetVehicleElimination();

					// Manage the EliminationAlert sound and redding screen.

					if (gameState->GamePlaySetup.DrivingMode == EDrivingMode::Elimination)
					{
						elimination.AlertTimer -= deltaSeconds;
						elimination.AlertTimer = FMath::Max(elimination.AlertTimer, 0.0f);

						// Check if we are in last position.

						if (gameMode->GetNumOpponentsLeft() - 1 == RacePosition)
						{
							// Don't play sound for the AI.

							if (PlayerVehicle->IsHumanPlayer() == true &&
								PlayerVehicle->IsCinematicCameraActive() == false)
							{
								float minCooldown = 0.15f; // In seconds
								float maxCooldown = 1.5f; // In seconds

								eliminationRatio = gameMode->GetEliminationRatio();

								if (eliminationRatio != 0.0f &&
									elimination.AlertTimer <= 0.0f)
								{
									PlayerVehicle->GetHUD().Warning(EHUDWarningSource::Elimination, 1.0f, 1.0f);

									elimination.AlertTimer = FMath::Lerp(maxCooldown, minCooldown, FMath::Sin(eliminationRatio * PI * 0.5f));

									// Play the sound.

									PlayerVehicle->ClientPlaySound(elimination.AlertSound);
								}
							}
						}
						else
						{
							// If we are not last set timer on 0.

							elimination.AlertTimer = 0.0f;
						}

						// Game stops when all opponents are destroyed.

						if (gameMode->NoOpponentsLeft() == true &&
							gameMode->GetNumOpponents() > 1)
						{
							RacePosition = gameMode->GetNumOpponentsLeft() - 1;

							PlayerComplete(true, true, false);
						}

						float ratio = FMathEx::GetSmoothingRatio(0.95f, deltaSeconds);

						elimination.Ratio = FMath::Lerp(eliminationRatio, elimination.Ratio, ratio);
					}

#pragma endregion GameModeElimination

					// We're in a lap-based kind of game mode, so let's handle that here.

					UpdateCheckpoints(false);
				}

				if (PlayerCompletionState >= EPlayerCompletionState::Complete)
				{
					GameFinishedAt = gameMode->GetRealTimeClock();
				}
			}
			else
			{
				if (gameState->IsGameModeRace() == true)
				{
					// We're in a lap-based kind of game mode the update the checkpoints even if the game
					// has finished for this player as the cinematic camera relies on the progress information.

					UpdateCheckpoints(false);
				}
			}
		}
	}
}

/**
* Update the checkpoints for this player race state to determine their progress
* around the track.
***********************************************************************************/

void FPlayerRaceState::UpdateCheckpoints(bool ignoreCheckpointSize)
{
}

/**
* Complete the event for the player.
***********************************************************************************/

void FPlayerRaceState::PlayerComplete(bool setCompletionStatus, bool gameComplete, bool estimateRaceTime, EPlayerCompletionState completionState)
{
	bool disqualified = PlayerCompletionState == EPlayerCompletionState::Disqualified;

	if (PlayerCompletionState != EPlayerCompletionState::Complete &&
		PlayerCompletionState != EPlayerCompletionState::Abandoned)
	{
		if (estimateRaceTime == true)
		{
			UE_LOG(GripLog, Log, TEXT("FPlayerRaceState::PlayerComplete estimating event result for %s"), *PlayerVehicle->GetPlayerName(false, false));
		}

		APlayGameMode* gameMode = APlayGameMode::Get(PlayerVehicle);
		UGlobalGameState* gameState = UGlobalGameState::GetGlobalGameState(PlayerVehicle);

		if (setCompletionStatus == true &&
			disqualified == false)
		{
			PlayerCompletionState = completionState;
		}

		if (gameState->GamePlaySetup.DrivingMode != EDrivingMode::Elimination)
		{
			RacePosition = gameMode->CollectFinishingRacePosition();
		}

		if (estimateRaceTime == true)
		{
			float gameClock = gameMode->GetRealTimeGameClock();

			if (gameState->IsGameModeLapBased() == true)
			{
				float progress = PlayerVehicle->GetEventProgress();

				if (progress > KINDA_SMALL_NUMBER)
				{
					RaceTime /= progress;
				}

				RaceTime = FMath::Max(RaceTime, gameClock);
			}
			else if (gameState->GamePlaySetup.DrivingMode == EDrivingMode::Elimination)
			{
				int32 index = (gameMode->GetNumOpponents() - RacePosition) + 1;

				RaceTime = (index * GRIP_ELIMINATION_SECONDS) + FMath::FRandRange(-0.2f, 0.2f);
			}
		}
	}
}

/**
* Add points to the player's total if the player's game hasn't ended.
***********************************************************************************/

bool FPlayerRaceState::AddPoints(int32 numPoints)
{
	if (IsAccountingClosed() == false)
	{
		// Only register points if the game is in-play. If accounting is closed
		// then we're too late for points now.

		NumInGamePoints += numPoints;
		NumTotalPoints += numPoints;

		return true;
	}

	return false;
}
