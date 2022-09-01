/**
*
* A time sharing clock, for spreading the load on the CPU.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
***********************************************************************************/

#pragma once

#include "CoreMinimal.h"
#include "Misc/App.h"

/**
* A time sharing clock structure, for spreading the load on the CPU.
***********************************************************************************/

struct FTimeShareClock
{
public:

	// Construct passing the time period between ticks in seconds.
	FTimeShareClock(double regularity = 1.0)
		: Regularity(regularity)
	{ }

	// Tick the structure, to manage time and determine if the caller should tick this frame.
	void Tick(int32 index, int32 numIndices, double time = -1.0)
	{
		TickNow = true;

		if (time < 0.0)
		{
			time = FApp::GetCurrentTime();
		}

		if (numIndices > 0)
		{
			if (NextTick <= time)
			{
				double offset = (Regularity / numIndices) * index;

				NextTick = (time - fmod(time, Regularity)) + Regularity + offset;
			}
			else
			{
				TickNow = false;
			}
		}
	}

	// Should we tick on this game frame?
	bool ShouldTickNow() const
	{ return TickNow; }

private:

	// The time period between ticks in seconds.
	double Regularity = 1.0;

	// At what clock time the next Tick should occur.
	double NextTick = 0.0;

	// Should we tick on this game frame?
	bool TickNow = true;
};
