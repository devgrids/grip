/**
*
* List of values against time and common operations.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* This is a highly-optimized version of a simple queue where we add to the end
* and remove from the beginning. This is used in a lot of places in the game
* to we've replaced the old TArray with something much faster in this use case.
*
* It uses a circular buffer and handles all of the indexing internally so you don't
* need to be aware of read and write cursors, just index it as normal.
*
* Values are added to the internal list at a fixed interval, not always when you
* ask for as that may be far too frequent. In this case, you can choose to sum the
* values you added since one was last committed to the list, or average them, or
* just take the last one as it is a bit like point-sampling. Averaging is normally
* the case.
*
* There are many functions to query the list of values over recent time to grab
* the mean value, or the sum, that kind of thing. This is great for examining a
* property over time, rather than instantaneously at the current time.
*
***********************************************************************************/

#pragma once

#include "system/mathhelpers.h"

template<typename ValueType>
class GRIP_API TTimedValueList
{
public:

	struct FTimeValue
	{
		FTimeValue(float time = 0.0f, ValueType value = ValueType(0.0f))
			: Time(time)
			, Value(value)
		{ }

		bool operator < (const FTimeValue& other)
		{ return (Value < other.Value); }

		float Time;
		ValueType Value;
	};

	// Construct a timed valued list.
	TTimedValueList(int32 maxSeconds = 1, int32 samplesPerSecond = 60, bool averageSamples = true, bool sumSamples = false)
	{ Reset(maxSeconds, samplesPerSecond, averageSamples, sumSamples); }

	// Construct a timed valued list from another list.
	TTimedValueList(const TTimedValueList& other)
	{ *this = other; }

	~TTimedValueList()
	{ if (Values != nullptr) delete Values; Values = nullptr; }

	// Reset a timed valued list, effectively constructing it.
	void Reset(int32 maxSeconds = 1, int32 samplesPerSecond = 60, bool averageSamples = true, bool sumSamples = false)
	{
		MaxSeconds = maxSeconds;
		MaxValues = maxSeconds * ((samplesPerSecond > 0) ? samplesPerSecond : 1000);
		IndexMask = FMathEx::GetPower2(MaxValues);
		NumValues = 0;
		ReadCursor = 0;
		WriteCursor = 0;
		SecondsPerSample = (samplesPerSecond > 0) ? 1.0f / samplesPerSecond : -1.0f;
		LastTime = 0;
		LastValue = ValueType(0.0f);
		SumStart = -1;
		SumValues = ValueType(0.0f);
		NumSumValues = 0;
		AverageSamples = averageSamples;
		SumSamples = sumSamples;
		Full = false;

		if (Values != nullptr)
		{
			delete Values;
		}

		Values = new FTimeValue[IndexMask--];
	}

	// Add a value to the value list.
	void AddValue(float time, ValueType value)
	{
		LastTime = time;
		LastValue = value;

		if (SumStart < 0)
		{
			SumStart = time;
			SumValues = ValueType(0.0f);
			NumSumValues = 0;
		}

		SumValues += value;
		NumSumValues++;

		do
		{
			if (SecondsPerSample <= 0 ||
				time > SumStart + SecondsPerSample)
			{
				if (SecondsPerSample > 0)
				{
					if (NumValues >= MaxValues)
					{
						Full = true;
						NumValues = MaxValues - 1;
					}
				}
				else
				{
					while (NumValues > 0 &&
						time - (*this)[0].Time > MaxSeconds)
					{
						Full = true;
						NumValues--;
						SetReadCursor();
					}
				}

				NumValues++;
				WriteCursor = (WriteCursor + 1) & IndexMask;
				SetReadCursor();

				if (SumSamples == true)
				{
					Values[WriteCursor] = FTimeValue(time, SumValues);
				}
				else if (AverageSamples == true &&
					NumSumValues > 0)
				{
					Values[WriteCursor] = FTimeValue(SumStart, SumValues * (1.0f / NumSumValues));
				}
				else
				{
					Values[WriteCursor] = FTimeValue(time, value);
				}

				SumValues = ValueType(0.0f);
				NumSumValues = 0;
				SumStart += SecondsPerSample;
			}
		}
		while (SecondsPerSample > 0 && time > SumStart + SecondsPerSample);
	}

	// Get the last time added to the list.
	float GetLastTime() const
	{ return LastTime; }

	// Get the last value added to the list.
	ValueType GetLastValue() const
	{ return LastValue; }

	// Get the minimum value of all the values in the list.
	ValueType GetMinValue(float since = -1.0f) const
	{
		ValueType min = ValueType(0.0f);

		if (NumValues > 0)
		{
			if (since < 0.0f)
			{
				for (int32 i = 0; i < NumValues; i++)
				{
					ValueType value = (*this)[i].Value;

					if (i == 0 ||
						min > value)
					{
						min = value;
					}
				}
			}
			else
			{
				int32 lastValue = NumValues - 1;

				for (int32 i = lastValue; i >= 0; i--)
				{
					const FTimeValue& element = (*this)[i];

					if (element.Time < since)
					{
						break;
					}

					if (i == lastValue ||
						min > element.Value)
					{
						min = element.Value;
					}
				}
			}
		}

		return min;
	}

	// Get the maximum value of all the values in the list.
	ValueType GetMaxValue(float since = -1.0f) const
	{
		ValueType max = ValueType(0.0f);

		if (NumValues > 0)
		{
			if (since < 0.0f)
			{
				for (int32 i = 0; i < NumValues; i++)
				{
					ValueType value = (*this)[i].Value;

					if (i == 0 ||
						max < value)
					{
						max = value;
					}
				}
			}
			else
			{
				int32 lastValue = NumValues - 1;

				for (int32 i = lastValue; i >= 0; i--)
				{
					const FTimeValue& element = (*this)[i];

					if (element.Time < since)
					{
						break;
					}

					if (i == lastValue ||
						max < element.Value)
					{
						max = element.Value;
					}
				}
			}
		}

		return max;
	}

	// Get the mean average value of all the values in the list.
	ValueType GetMeanValue(float since = -1.0f) const
	{
		ValueType sum = ValueType(0.0f);
		int32 numSummed = 0;

		if (NumValues > 0)
		{
			if (since < 0.0f)
			{
				for (int32 i = 0; i < NumValues; i++)
				{
					sum += (*this)[i].Value; numSummed++;
				}
			}
			else
			{
				for (int32 i = NumValues - 1; i >= 0; i--)
				{
					const FTimeValue& element = (*this)[i];

					if (element.Time < since)
					{
						break;
					}

					sum += element.Value; numSummed++;
				}
			}
		}

		return (numSummed > 0) ? sum / numSummed : ValueType(0.0f);
	}

	// Get the unfluttered value of all the values in the list.
	// This attempts to remove any hysteresis recorded in the values in the list and smooth out the result.
	ValueType GetUnflutteredValue(float since = -1.0f, bool higher = false) const
	{
		ValueType sumLow = ValueType(0.0f), sumHigh = ValueType(0.0f);
		int32 numSummedLow = 0, numSummedHigh = 0, numSwitches = 0, switchPosition = -1;

		if (NumValues > 0)
		{
			for (int32 i = NumValues - 1; i >= 0; i--)
			{
				const FTimeValue& element = (*this)[i];

				if (since >= 0.0f &&
					element.Time < since)
				{
					break;
				}

				if ((*this)[i].Value < 0.0f)
				{
					if (switchPosition == 1)
					{
						numSwitches++;
					}

					switchPosition = 0;

					sumLow += element.Value; numSummedLow++;
				}
				else
				{
					if (switchPosition == 0)
					{
						numSwitches++;
					}

					switchPosition = 1;

					sumHigh += element.Value; numSummedHigh++;
				}
			}
		}

		if (numSwitches == 0)
		{
			return ((numSummedHigh > 0) ? sumHigh / numSummedHigh : ValueType(0.0f)) + ((numSummedLow > 0) ? sumLow / numSummedLow : ValueType(0.0f));
		}
		else if (higher == true)
		{
			return (numSummedHigh > 0) ? sumHigh / numSummedHigh : ValueType(0.0f);
		}
		else
		{
			return (numSummedLow > 0) ? sumLow / numSummedLow : ValueType(0.0f);
		}
	}

	// Get the mean average value of all the values in the list.
	ValueType GetAbsMeanValue(float since = -1.0f) const
	{
		ValueType sum = ValueType(0.0f);
		int32 numSummed = 0;

		if (NumValues > 0)
		{
			if (since < 0.0f)
			{
				for (int32 i = 0; i < NumValues; i++)
				{
					sum += FMath::Abs((*this)[i].Value); numSummed++;
				}
			}
			else
			{
				for (int32 i = NumValues - 1; i >= 0; i--)
				{
					const FTimeValue& element = (*this)[i];

					if (element.Time < since)
					{
						break;
					}

					sum += FMath::Abs(element.Value); numSummed++;
				}
			}
		}

		return (numSummed > 0) ? sum / numSummed : ValueType(0.0f);
	}

	// Get the mean average value of all the values in the list scaled by the number of
	// values recorded in the list vs its maximum size.
	ValueType GetScaledMeanValue() const
	{
		ValueType sum = ValueType(0.0f);
		int32 numSummed = 0;

		if (NumValues > 0)
		{
			for (int32 i = 0; i < NumValues; i++)
			{
				sum += (*this)[i].Value; numSummed++;
			}
		}

		if (numSummed > 0 &&
			MaxValues > 0)
		{
			return (sum / numSummed) * (float)NumValues / (float)MaxValues;
		}
		else
		{
			return ValueType(0.0f);
		}
	}

	// Get the mean average value of all the values in the list scaled by the number of
	// values recorded in the list vs its maximum size.
	ValueType GetAbsScaledMeanValue() const
	{
		ValueType sum = ValueType(0.0f);
		int32 numSummed = 0;

		if (NumValues > 0)
		{
			for (int32 i = 0; i < NumValues; i++)
			{
				sum += FMath::Abs((*this)[i].Value); numSummed++;
			}
		}

		if (numSummed > 0 &&
			MaxValues > 0)
		{
			return sum / numSummed * (float)NumValues / (float)MaxValues;
		}
		else
		{
			return ValueType(0.0f);
		}
	}

	// Get the sum value of all the values in the list.
	ValueType GetSumValue(float since = -1.0f) const
	{
		ValueType sum = ValueType(0.0f);

		if (NumValues > 0)
		{
			if (since < 0.0f)
			{
				for (int32 i = 0; i < NumValues; i++)
				{
					sum += (*this)[i].Value;
				}
			}
			else
			{
				for (int32 i = NumValues - 1; i >= 0; i--)
				{
					const FTimeValue& element = (*this)[i];

					if (element.Time < since)
					{
						break;
					}

					sum += element.Value;
				}
			}
		}

		return sum;
	}

	// Get the sum value of all the values in the list.
	ValueType GetAbsSumValue(float since = -1.0f) const
	{
		ValueType sum = ValueType(0.0f);

		if (NumValues > 0)
		{
			if (since < 0.0f)
			{
				for (int32 i = 0; i < NumValues; i++)
				{
					sum += FMath::Abs((*this)[i].Value);
				}
			}
			else
			{
				for (int32 i = NumValues - 1; i >= 0; i--)
				{
					const FTimeValue& element = (*this)[i];

					if (element.Time < since)
					{
						break;
					}

					sum += FMath::Abs(element.Value);
				}
			}
		}

		return sum;
	}

	// Get the value at a particular time in the list.
	ValueType GetValueAt(float at) const
	{
		int32 i = NumValues - 1;

		for (; i > 0; i--)
		{
			const FTimeValue& element = (*this)[i];

			if (element.Time < at)
			{
				i++;
				break;
			}
		}

		return (i < NumValues) ? (*this)[i].Value : ValueType(0.0f);
	}

	// Get the difference between the value given and the value stored at at, and divide that
	// by the time difference between the clock given and time stored at at, thus the change
	// in the values that would occur over one second of time, regardless of how much time
	// we're examining.
	ValueType DifferenceFromPerSecond(float at, float clock, float value) const
	{
		int32 i = NumValues - 1;

		for (; i > 0; i--)
		{
			const FTimeValue& element = (*this)[i];

			if (element.Time < at)
			{
				i++;
				break;
			}
		}

		for (; i < NumValues; i++)
		{
			const FTimeValue& element = (*this)[i];

			if (element.Time >= at)
			{
				float timeDifference = (clock - element.Time);

				if (timeDifference > KINDA_SMALL_NUMBER)
				{
					return (value - element.Value) / timeDifference;
				}
				else
				{
					return value - element.Value;
				}
			}
		}

		float timeDifference = (clock - LastTime);

		if (timeDifference > KINDA_SMALL_NUMBER)
		{
			return (value - LastValue) / timeDifference;
		}
		else
		{
			return value - LastValue;
		}
	}

	// Get the time range of the values in the list.
	float TimeRange() const
	{
		if (NumValues > 0)
		{
			return (*this)[NumValues - 1].Time - (*this)[0].Time;
		}
		else
		{
			return 0.0f;
		}
	}

	// Clear the list of all recorded values.
	void Clear()
	{
		Full = false;
		NumValues = ReadCursor = WriteCursor = 0;
	}

	// Clear the list of all recorded values with a Time < time.
	void Clear(float time)
	{
		while (NumValues > 0 &&
			(*this)[0].Time < time)
		{
			Full = false;
			NumValues--;
			SetReadCursor();
		}
	}

	// Get the number of values in the list.
	int32 GetNumValues() const
	{ return NumValues; }

	// Get the maximum number of values in the list.
	int32 GetMaxValues() const
	{ return MaxValues; }

	// Is the list full? Meaning is it storing its maximum capacity of of values yet?
	bool IsFull() const
	{ return Full; }

	// Note that index 0 is the oldest value in the list and _numValues - 1 is the most recent.
	const FTimeValue& operator [] (int32 index) const
	{ return Values[(index + ReadCursor) & IndexMask]; }

	// Assign a TTimedValueList to this object.
	TTimedValueList<ValueType>& operator = (const TTimedValueList<ValueType>& other)
	{
		FMemory::Memcpy(this, &other, sizeof(other));

		int32 numValues = IndexMask + 1;

		Values = new FTimeValue[numValues];

		FMemory::Memcpy(Values, other.Values, numValues * sizeof(FTimeValue));

		return *this;
	}

private:

	// Set the read cursor for the list after adding or removing values.
	void SetReadCursor()
	{ if ((ReadCursor = WriteCursor - (NumValues - 1)) < 0) ReadCursor = (IndexMask + 1) + ReadCursor; }

	float MaxSeconds;

	int32 IndexMask;

	// The capacity of the circular buffer.
	int32 MaxValues;

	// The number of used values in the circular buffer.
	int32 NumValues;

	// The index of the first value in the notional array.
	int32 ReadCursor;

	// The index of the last value that was written.
	int32 WriteCursor;

	// How many seconds there is in a sample in the buffer.
	float SecondsPerSample;

	// The last time passed to AddValue.
	float LastTime;

	// The last value passed to AddValue.
	ValueType LastValue;

	// The time at which accruing started before storing into the list.
	float SumStart;

	// The sum of the values we're accruing before storing into the list.
	ValueType SumValues;

	// The number of summed values we're accruing before storing into the list.
	int32 NumSumValues;

	// Are we averaging samples that we're accruing before adding them to the list?
	bool AverageSamples;

	// Are we summing samples that we're accruing before adding them to the list?
	bool SumSamples;

	// Is the list full? Meaning is it storing its maximum capacity of of values yet?
	bool Full;

	// The circular buffer storing the values.
	FTimeValue* Values = nullptr;
};

// A timed value list for the float type.
typedef TTimedValueList<float> FTimedFloatList;

// A timed value list for the FVector type.
typedef TTimedValueList<FVector> FTimedVectorList;
