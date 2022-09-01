/**
*
* Math helper functions.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* A whole bunch of math helper functions and structures to be used in addition to
* the regular FMath namespace. Here, they're referenced by the FMathEx namespace
* instead, as the original namespace couldn't be extended.
*
***********************************************************************************/

#pragma once

#include "system/gameconfiguration.h"

namespace FMathEx
{

	/**
	* Returns +1 or -1 depending on relation of T to 0
	***********************************************************************************/

	template<class T>
	static CONSTEXPR FORCEINLINE T UnitSign(const T x)
	{ return (x >= (T)0) ? (T)+1 : (T)-1; }

	/**
	* Scale a smoothing ratio against delta seconds and the authored frames per second,
	* to give constant-rate smoothing no matter the frame rate.
	***********************************************************************************/

	FORCEINLINE float GetSmoothingRatio(float ratio, float deltaSeconds, float authoredFPS = 60.0f)
	{
		float timeRatio = deltaSeconds * authoredFPS;

		if ((ratio > 0.0f) &&
			(timeRatio > 0.0f))
		{
			return 1.0f / FMath::Pow(1.0f / ratio, timeRatio);
		}

		return ratio;
	}

	/**
	* Smooth a vector from one to another using a smoothing ratio and delta seconds.
	***********************************************************************************/

	FORCEINLINE FVector GetSmoothedVector(const FVector& from, const FVector& to, float smoothingRatio, float deltaSeconds)
	{ return FMath::Lerp(to, from, GetSmoothingRatio(smoothingRatio, deltaSeconds)); }

	/**
	* Interpolate a rotation from one to another using a ratio, always taking the direct
	* raw path.
	***********************************************************************************/

	inline FRotator RInterpToRaw(const FRotator& from, const FRotator& to, float ratio)
	{
		if (from == to ||
			ratio <= 0.0f)
		{
			return from;
		}

		FRotator difference = to - from;

		if (difference.IsNearlyZero() == true)
		{
			return to;
		}

		return (from + (difference * FMath::Min(ratio, 1.0f))).GetNormalized();
	}

	/**
	* Smooth a rotation from one to another using a smoothing ratio and delta seconds,
	* always taking the shortest path if pre-normalized.
	***********************************************************************************/

	inline FRotator GetSmoothedRotation(const FRotator& from, const FRotator& to, float deltaSeconds, float yawSmoothing, float pitchSmoothing, float rollSmoothing)
	{
		if (from == to)
		{
			return from;
		}

		FRotator difference = (to - from).GetNormalized();

		if (difference.IsNearlyZero() == true)
		{
			return to;
		}
		else
		{
			float yaw = difference.Yaw * (1.0f - GetSmoothingRatio(yawSmoothing, deltaSeconds));
			float pitch = difference.Pitch * (1.0f - GetSmoothingRatio(pitchSmoothing, deltaSeconds));
			float roll = difference.Roll * (1.0f - GetSmoothingRatio(rollSmoothing, deltaSeconds));

			return FRotator(from.Pitch + pitch, from.Yaw + yaw, from.Roll + roll).GetNormalized();
		}
	}

	/**
	* The aim here is to target things most in front of the vehicle, but taking
	* distance into account. This means that if target A was a greater angle away but
	* closer to the vehicle than target B which had a smaller angle, then target A may
	* be chosen in preference.
	*
	* Returns a number between 0 and 1 for a valid target, with 0 being perfect weight.
	* Returns -1 for an invalid target.
	***********************************************************************************/

	inline float TargetWeight(const FVector& fromPosition, const FVector& fromDirection, const FVector& targetPosition, float minDistance, float maxDistance, float maxCosAngle, bool weightAngle)
	{
		FVector targetDirection = targetPosition - fromPosition;
		float distance = targetDirection.Size();

		// Only consider targets within range.

		if (distance > minDistance &&
			distance < maxDistance)
		{
			targetDirection.Normalize();

			float dotProduct = FVector::DotProduct(fromDirection, targetDirection);

			// Only consider targets within the cone.

			if (dotProduct > maxCosAngle)
			{
				float degrees = FMath::Acos(dotProduct);
				float maxDegrees = FMath::Acos(maxCosAngle);
				float distanceFactor = (distance - minDistance) / (maxDistance - minDistance);

				// Cube the distance to prefer closer targets rather than less angle away.

				return ((weightAngle == true) ? (degrees / maxDegrees) : 1.0f) * (distanceFactor * distanceFactor * distanceFactor);
			}
		}

		return -1.0f;
	}

	/**
	* Return unnormalized angle x to the range 0 to +PI * 2 radians.
	***********************************************************************************/

	FORCEINLINE float GetNormalizedUnsignedRadians(float x)
	{
		x = FMath::Fmod(x, PI * 2.0f * FMathEx::UnitSign(x));

		if (x < 0.0f)
		{
			x = (PI * 2.0f) + x;
		}

		return x;
	}

	/**
	* Return unnormalized angle x to the range 0 to 360 degrees.
	***********************************************************************************/

	FORCEINLINE float GetNormalizedUnsignedDegrees(float x)
	{
		x = FMath::Fmod(x, 360.0f * FMathEx::UnitSign(x));

		if (x < 0.0f)
		{
			x = 360.0f + x;
		}

		return x;
	}

	/**
	* Return angle x to the range 0 to 360 degrees.
	***********************************************************************************/

	FORCEINLINE float GetUnsignedDegrees(float x)
	{
		checkSlow(x >= -180.0f && x <= 180.0f);

		if (x < 0.0f)
		{
			x = 360.0f + x;
		}

		return x;
	}

	/**
	* Return rotator to the range 0 to 360 degrees.
	***********************************************************************************/

	inline FRotator GetUnsignedDegrees(const FRotator& x)
	{
		FRotator result;

		result.Yaw = GetUnsignedDegrees(x.Yaw);
		result.Roll = GetUnsignedDegrees(x.Roll);
		result.Pitch = GetUnsignedDegrees(x.Pitch);

		return result;
	}

	/**
	* Get the difference between angle a and angle b, both in degrees, optionally
	* normalizing them first.
	***********************************************************************************/

	inline float GetUnsignedDegreesDifference(float a, float b, bool normalize = true)
	{
		if (normalize == true)
		{
			a = GetNormalizedUnsignedDegrees(a);
			b = GetNormalizedUnsignedDegrees(b);
		}
		else
		{
			a = GetUnsignedDegrees(a);
			b = GetUnsignedDegrees(b);
		}

		return FMath::Min(360.0f - FMath::Abs(a - b), FMath::Abs(a - b));
	}

	/**
	* Get the difference between rotator a and rotator b, both in degrees, optionally
	* normalizing them first.
	***********************************************************************************/

	inline FRotator GetUnsignedDegreesDifference(const FRotator& a, const FRotator& b, bool normalize = true)
	{
		FRotator result;

		result.Yaw = GetUnsignedDegreesDifference(a.Yaw, b.Yaw, normalize);
		result.Roll = GetUnsignedDegreesDifference(a.Roll, b.Roll, normalize);
		result.Pitch = GetUnsignedDegreesDifference(a.Pitch, b.Pitch, normalize);

		return result;
	}

	/**
	* Get the signed difference between angle a and angle b, both in degrees, optionally
	* normalizing them first.
	***********************************************************************************/

	inline float GetSignedDegreesDifference(float a, float b, bool normalize = true)
	{
		if (normalize == true)
		{
			a = GetNormalizedUnsignedDegrees(a);
			b = GetNormalizedUnsignedDegrees(b);
		}

		float result = b - a;

		const float p0 = 90.0f;
		const float p1 = 270.0f;
		const float p2 = 360.0f;

		if (a < p0 && b >= p1)
		{
			result = -(a + (p2 - b));
		}
		else if (b < p0 && a >= p1)
		{
			result = (b + (p2 - a));
		}

		return result;
	}

	/**
	* Get the signed difference between rotator a and rotator b, both in degrees,
	* optionally normalizing them first.
	***********************************************************************************/

	inline FRotator GetSignedDegreesDifference(const FRotator& a, const FRotator& b, bool normalize = true)
	{
		FRotator result;

		result.Yaw = GetSignedDegreesDifference(a.Yaw, b.Yaw, normalize);
		result.Roll = GetSignedDegreesDifference(a.Roll, b.Roll, normalize);
		result.Pitch = GetSignedDegreesDifference(a.Pitch, b.Pitch, normalize);

		return result;
	}

	/**
	* Get the next power of 2 up from a given value.
	***********************************************************************************/

	inline int32 GetPower2(int32 value)
	{
		for (int32 i = 0; i < 32; i++)
		{
			if (value <= (1 << i))
			{
				return (1 << i);
			}
		}

		checkSlow(0);

		return 0;
	}

	/**
	* DegreesToDotProduct returns +1 ... -1 for angles 0 ... 180.
	***********************************************************************************/

	FORCEINLINE float DegreesToDotProduct(float degrees)
	{ return FMath::Cos(FMath::DegreesToRadians(degrees)); }

	/**
	* ConeDegreesToDotProduct returns +1 ... -1 for angles 0 ... 360.
	***********************************************************************************/

	FORCEINLINE float ConeDegreesToDotProduct(float degrees)
	{ return DegreesToDotProduct(degrees * 0.5f); }

	/**
	* DotProductToDegrees returns 0 ... 180 for dot product +1 ... -1.
	***********************************************************************************/

	FORCEINLINE float DotProductToDegrees(float dotProduct)
	{ return FMath::RadiansToDegrees(FMath::Acos(dotProduct)); }

	/**
	* Convert to meters from centimeters.
	***********************************************************************************/

	template<typename T>
	FORCEINLINE T CentimetersToMeters(const T& value)
	{ return value * (1.0f / 100.0f); }

	/**
	* Convert to centimeters from meters.
	***********************************************************************************/

	template<typename T>
	FORCEINLINE T MetersToCentimeters(const T& value)
	{ return value * 100.0f; }

	/**
	* Convert to meters per second from kilometers per hour.
	***********************************************************************************/

	template<typename T>
	FORCEINLINE T KilometersPerHourToMetersPerSecond(const T& value)
	{ return value * (1.0f / 3.6f); }

	/**
	* Convert to centimeters per second from kilometers per hour.
	***********************************************************************************/

	template<typename T>
	FORCEINLINE T KilometersPerHourToCentimetersPerSecond(const T& value)
	{ return MetersToCentimeters(value * (1.0f / 3.6f)); }

	/**
	* Convert to kilometers per hour from meters per second.
	***********************************************************************************/

	template<typename T>
	FORCEINLINE T MetersPerSecondToKilometersPerHour(const T& value)
	{ return (value * 3.6f); }

	/**
	* Convert to kilometers per hour from centimeters per second.
	***********************************************************************************/

	template<typename T>
	FORCEINLINE T CentimetersPerSecondToKilometersPerHour(const T& value)
	{ return (CentimetersToMeters(value) * 3.6f); }

	/**
	* Ease into a value at a particular easing rate starting slowly.
	***********************************************************************************/

	FORCEINLINE float EaseIn(float ratio, float easeRate = 2.0f)
	{ return FMath::Pow(ratio, easeRate); }

	/**
	* Ease out of a value at a particular easing rate ending slowly.
	***********************************************************************************/

	FORCEINLINE float EaseOut(float ratio, float easeRate = 2.0f)
	{ return 1.0f - FMath::Pow(1.0f - ratio, easeRate); }

	/**
	* Ease out of and into a value at a particular easing rate, starting and ending slowly.
	***********************************************************************************/

	FORCEINLINE float EaseInOut(float ratio, float easeRate = 2.0f)
	{ return (ratio < 0.5f) ? 0.5f * FMath::Pow(2.0f * ratio, easeRate) : 1.0f - (0.5f * FMath::Pow(2.0f * (1.0f - ratio), easeRate)); }

	/**
	* Convert the differences between two locations to a normal.
	***********************************************************************************/

	FORCEINLINE FVector LocationsToNormal(const FVector& from, const FVector& to)
	{ FVector v0 = to - from; v0.Normalize(); return v0; }

	/**
	* Get the power of a potentially negative number.
	***********************************************************************************/

	FORCEINLINE float NegativePow(float a, float b)
	{ return FMath::Pow(FMath::Abs(a), b) * FMathEx::UnitSign(a); }

	/**
	* Clamp a rotation moving from one point to another by a maximum degrees change for
	* this particular call in pitch and yaw.
	***********************************************************************************/

	inline FRotator ClampRotationDifference(const FRotator& from, FRotator to, float maxDegrees, bool& clamped)
	{
		clamped = false;

		FVector toInFromLocal = from.UnrotateVector(to.Vector()); toInFromLocal.X = 0.0f;
		FVector maxRotation = FRotator(maxDegrees, 0.0f, 0.0f).Vector();
		float differenceSize = toInFromLocal.SizeSquared();

		if (toInFromLocal.X < 0.0f ||
			differenceSize > (maxRotation.Z * maxRotation.Z))
		{
			clamped = true;

			toInFromLocal.Normalize();
			toInFromLocal *= maxRotation.Z;
			toInFromLocal.X = FMath::Sqrt(1.0f - toInFromLocal.SizeSquared());

			check(FMath::Abs(1.0f - toInFromLocal.Size()) < 0.01f);

			to = from.RotateVector(toInFromLocal).Rotation();
		}

		return to;
	}

	/**
	* Gravitate a value to a target.
	***********************************************************************************/

	FORCEINLINE float GravitateToTarget(float value, float target, float delta)
	{ if (value > target) value = FMath::Max(value - delta, target); else if (value < target) value = FMath::Min(value + delta, target); return value; }

	/**
	* Gravitate a value up to a target.
	***********************************************************************************/

	FORCEINLINE float GravitateUpToTarget(float value, float target, float delta)
	{ if (value > target) value = target; else if (value < target) value = FMath::Min(value + delta, target); return value; }

	/**
	* Gravitate a value down to a target.
	***********************************************************************************/

	FORCEINLINE float GravitateDownToTarget(float value, float target, float delta)
	{ if (value < target) value = target; else if (value > target) value = FMath::Max(value - delta, target); return value; }

	/**
	* Get the squared distance of the line from a point in space. pointOnLine will
	* return the nearest point on the line in parametric form; 0 ... 1 encompassing
	* the line length, less than zero before the start of the line and greater than
	* one if over the end of the line.
	***********************************************************************************/

	inline float PointToLineDistanceSquared(const FVector& origin, const FVector& direction, const FVector& point, float& pointOnLine)
	{
		pointOnLine = 0.0f;

		FVector difference = point - origin;

		// Calculate the scalar for the nearest point on the line to the point that we
		// are comparing.

		float lengthSquared = direction.SizeSquared();

		if (lengthSquared > KINDA_SMALL_NUMBER)
		{
			pointOnLine = FVector::DotProduct(direction, difference) / lengthSquared;

			if (pointOnLine > 0.0f)
			{
				FVector lineVector = direction;

				if (pointOnLine < 1.0f)
				{
					// Get the nearest point on the line to the point that we are comparing and
					// return the squared distance between them.

					lineVector *= pointOnLine;
				}

				difference -= lineVector;
			}
		}

		return difference.SizeSquared();
	}

	/**
	* Get the distance of the line from a line in space.
	***********************************************************************************/

	FORCEINLINE float PointToLineDistance(const FVector& origin, const FVector& direction, const FVector& point, float& pointOnLine)
	{ return FMath::Sqrt(PointToLineDistanceSquared(origin, direction, point, pointOnLine)); }

	/**
	* Check if the ray intersects this plane.
	***********************************************************************************/

	inline bool RayIntersectsPlane(const FVector& rayOrigin, const FVector& rayDirection, const FVector& planeOrigin, const FVector& planeDirection, FVector& intersection)
	{
		float cosAlpha = FVector::DotProduct(rayDirection, planeDirection);

		if (cosAlpha >= 0.0f)
		{
			return false;
		}

		FVector difference = planeOrigin - rayOrigin;
		float dotDifference = FVector::DotProduct(planeDirection, difference);
		float delta = dotDifference / cosAlpha;

		if (delta < 0.0f)
		{
			return false;
		}

		intersection = rayOrigin + (rayDirection * delta);

		return true;
	}

	/**
	* Calculate the right and up vectors for a forward vector and notional up vector.
	***********************************************************************************/

	inline bool GetRightUpVectors(const FVector& forward, FVector& right, FVector& up)
	{
		FVector other(up);
		bool valid = true;

		if (FMath::Abs(FVector::DotProduct(forward, other)) > 1.0f - KINDA_SMALL_NUMBER)
		{
			// Should almost never happen, but do something useful just so that we don't
			// get completely spastic vectors being returned.

			valid = false;
			other = FVector(up.Y, up.Z, up.X);
		}

		right = FVector::CrossProduct(other, forward);
		right.Normalize();

		up = FVector::CrossProduct(forward, right);

		return valid;
	}

	/**
	* Calculate a quaternion  from a forward vector and notional up vector.
	***********************************************************************************/

	inline bool GetQuaternionFromForwardUp(const FVector& forward, FVector up, FQuat& quaternion)
	{
		FVector right = FVector::ZeroVector;
		bool result = FMathEx::GetRightUpVectors(forward, right, up);
		FMatrix matrix = FMatrix(forward, right, up, FVector::ZeroVector);

		quaternion = FQuat(matrix);

		return result;
	}

	/**
	* Calculate a rotation from a forward vector and notional up vector.
	***********************************************************************************/

	inline bool GetRotationFromForwardUp(const FVector& forward, FVector up, FRotator& rotation)
	{
		FVector right = FVector::ZeroVector;
		bool result = FMathEx::GetRightUpVectors(forward, right, up);
		FMatrix matrix = FMatrix(forward, right, up, FVector::ZeroVector);

		rotation = FQuat(matrix).Rotator();

		return result;
	}

	/**
	* Get the ratio of a value between a minimum and maximum range, optionally clamped
	* to the 0 ... 1 range.
	***********************************************************************************/

	inline float GetRatio(float value, float minimum, float maximum, bool clamp = true)
	{
		if (maximum < minimum)
		{
			float temp = maximum;
			maximum = minimum;
			minimum = temp;
		}

		if (value > maximum && clamp == true)
		{
			return 1.0f;
		}
		else if (value > minimum)
		{
			return (value - minimum) / (maximum - minimum);
		}
		else
		{
			return 0.0f;
		}
	}

	/**
	* Get the inverse ratio of a value between a minimum and maximum range, clamped to
	* the 0 ... 1 range.
	***********************************************************************************/

	FORCEINLINE float GetInverseRatio(float value, float minimum, float maximum)
	{ return 1.0f - GetRatio(value, minimum, maximum, true); }

	/**
	* Structure used for oscillation.
	***********************************************************************************/

	struct FOscillator
	{
		// Clamp time to 1, meaning to PI * 2 for the Sin wave.
		bool ClampToPI2 = true;

		// The amplitude of the oscillator.
		float Amplitude = 0.0f;

		// The frequency of the oscillator.
		float Frequency = 1.0f;

		// The current time for the oscillator.
		float Time = 0.0f;

		// Construct the oscillator with the given amplitude and frequency.
		FOscillator(float amplitude = 0.0f, float frequency = 1.0f, bool clampToPI2 = true)
			: ClampToPI2(clampToPI2)
			, Amplitude(amplitude)
			, Frequency(frequency)
			, Time(FMath::FRand())
		{ }

		void Tick(float deltaSeconds)
		{ Time += deltaSeconds * Frequency; if (ClampToPI2 == true) Time = FMath::Fmod(Time, 1.0f); }

		float GetValue() const
		{ return FMath::Sin(Time * PI * 2.0f) * Amplitude; }
	};

	/**
	* Structure used for the generation of sine-based noise.
	***********************************************************************************/

	struct FSineNoise
	{
		// Use 6 oscillators for a detailed noise pattern.
		static const int32 NumOscillators = 6;

		// Construct the sine noise, indicating whether you want predominantly low frequency results from it.
		FSineNoise(bool lowFrequency = false)
		{
			static float AmplitudesLow[NumOscillators] = { 1.0f, 0.5f, 0.5f, 0.05f, 0.05f, 0.05f };
			static float AmplitudesHigh[NumOscillators] = { 1.0f, 1.2f, 0.5f, 0.4f, 0.3f, 0.2f };
			static float Frequencies[NumOscillators] = { 1, 1.25, 1.5, 3, 6, 12 };

			for (int32 i = 0; i < NumOscillators; i++)
			{
				Oscillators[i] = FOscillator(((lowFrequency) ? AmplitudesLow[i] : AmplitudesHigh[i]), Frequencies[i], true);
			}
		}

		// Tick the sine noise with time.
		void Tick(float deltaSeconds)
		{
			for (FOscillator& oscillator : Oscillators)
			{
				oscillator.Tick(deltaSeconds);
			}
		}

		// Get the current value of the sine noise.
		float GetValue()
		{
			float sum = 0.0f;

			for (FOscillator& oscillator : Oscillators)
			{
				sum += oscillator.GetValue();
			}

			return sum;
		}

		// The oscillators from which to form noise.
		FOscillator Oscillators[NumOscillators];
	};

	/**
	* Very fast random number generator class.
	***********************************************************************************/

	struct FRandomFast
	{
	public:

		static const int32 MAX_RANDOM = 0x7fff;

		FRandomFast() = default;

		FRandomFast(uint32 seed) : Root(seed)
		{ }

		// Conversion to float type, typically a left hand assignment
		// returns a random float between 0.0f and 1.0f (1.0f exclusive)
		operator float();

		// Conversion to int32 type, typically a left hand assignment
		// returns a random int32 between 0 and MAX_RANDOM (MAX_RANDOM inclusive)
		operator int32()
		{ return (int32)(float(*this) * (MAX_RANDOM + 1)); }

		// RHS Assignment operator, allows the seed to be set
		void operator = (uint32 seed)
		{ Root = seed; }

		// return a float scaled by an absolute scalar (rhs exclusive)
		float operator * (float rhs)
		{ return float(*this) * rhs; }

		// return a int32 scaled by an absolute scalar (rhs exclusive)
		int32 operator * (int32 rhs)
		{ return (int32)(float(*this) * rhs); }

		// return a uint32 scaled by an absolute scalar (rhs exclusive)
		uint32 operator * (uint32 rhs)
		{ return (*this) * int32(rhs); }

		// return a float scaled by a signed range (rhs exclusive)
		float operator ^ (float rhs)
		{ return (float(*this) * rhs * 2.0f) - rhs; }

		// return a int32 scaled by a signed range (rhs exclusive)
		int32 operator ^ (int32 rhs)
		{ return (int32)((float(*this) * rhs * 2.0f) - rhs); }

		// return a int32 scaled by a signed range (rhs exclusive)
		int32 operator ^ (uint32 rhs)
		{ return (*this) ^ int32(rhs); }

	private:

		// The current root for generating random numbers
		uint32 Root = 0;
	};

	/**
	* Conversion to float type, typically a left hand assignment
	* returns a random float between 0.0f and 1.0f (1.0f exclusive)
	***********************************************************************************/

	FORCEINLINE FRandomFast::operator float()
	{
		Root = 1664525L * Root + 1013904223L;

		uint32 itemp = 0x3f800000 | (0x007fffff & Root);

		return (*(float*)&itemp) - 1.0f;
	}

	/**
	* Slower random number generator class, using the Mersenne Twister algorithm.
	***********************************************************************************/

	struct FRandomSlow
	{
	public:

		static const int32 MAX_RANDOM = 0x7fff;

		FRandomSlow()
		{ *this = 5489; }

		FRandomSlow(uint32 seed)
		{ *this = seed; }

		// Conversion to float type, typically a left hand assignment
		// returns a random float between 0.0f and 1.0f (1.0f exclusive)
		operator float()
		{ return Random() * (1.0f / 4294967750.0f); }

		// Conversion to int32 type, typically a left hand assignment
		// returns a random int32 between 0 and MAX_RANDOM (MAX_RANDOM inclusive)
		operator int32()
		{ return Random() & MAX_RANDOM; }

		// RHS Assignment operator, allows the seed to be set
		void operator = (uint32 seed)
		{ mt[0] = seed & 0xffffffff; for (mti = 1; mti < N; mti++) mt[mti] = (69069 * mt[mti - 1]) & 0xffffffff; }

		// return a float scaled by an absolute scalar (rhs exclusive)
		float operator * (float rhs)
		{ return float(*this) * rhs; }

		// return a int32 scaled by an absolute scalar (rhs exclusive)
		int32 operator * (int32 rhs)
		{ return (Random() % ((rhs < 0) ? -rhs : rhs)); }

		// return a uint32 scaled by an absolute scalar (rhs exclusive)
		uint32 operator * (uint32 rhs)
		{ return Random() % rhs; }

		// return a float scaled by a signed range (rhs exclusive)
		float operator ^ (float rhs)
		{ return (float(*this) * rhs * 2.0f) - rhs; }

		// return a int32 scaled by a signed range (rhs exclusive)
		int32 operator ^ (int32 rhs)
		{ return (*this ^ uint32((rhs < 0) ? -rhs : rhs)); }

		// return a int32 scaled by a signed range (rhs exclusive)
		int32 operator ^ (uint32 rhs)
		{ return ((*this) * rhs * 2) - rhs; }

	private:

		// Generate a random number.
		uint32 Random();

		enum GeGenerationConstants
		{
			N = 624,
			M = 397,

			// constant vector a
			MATRIX_A = 0x9908b0df,

			// most significant w-r bits
			UPPER_MASK = 0x80000000,

			// least significant r bits
			LOWER_MASK = 0x7fffffff,

			// Tempering parameters
			TEMPERING_MASK_B = 0x9d2c5680,
			TEMPERING_MASK_C = 0xefc60000,
			TEMPERING_SHIFT_U = 11,
			TEMPERING_SHIFT_S = 7,
			TEMPERING_SHIFT_T = 15,
			TEMPERING_SHIFT_L = 18
		};

		// The array for the state vector
		uint32 mt[N];
		int32 mti = N + 1;
	};

	/**
	* Generate a random number.
	***********************************************************************************/

	inline uint32 FRandomSlow::Random()
	{
		uint32 result;
		static uint32 mag01[2] = { 0x0, (uint32)MATRIX_A };

		// Generate N words at one time.

		if (mti >= N)
		{
			int32 kk;

			if (mti == N + 1)
			{
				*this = 5489;
			}

			for (kk = 0; kk < N - M; kk++)
			{
				result = (mt[kk] & UPPER_MASK) | (mt[kk + 1] & LOWER_MASK);
				mt[kk] = mt[kk + M] ^ (result >> 1) ^ mag01[result & 0x1];
			}

			for (; kk < N - 1; kk++)
			{
				result = (mt[kk] & UPPER_MASK) | (mt[kk + 1] & LOWER_MASK);
				mt[kk] = mt[kk + (M - N)] ^ (result >> 1) ^ mag01[result & 0x1];
			}

			result = (mt[N - 1] & UPPER_MASK) | (mt[0] & LOWER_MASK);
			mt[N - 1] = mt[M - 1] ^ (result >> 1) ^ mag01[result & 0x1];

			mti = 0;
		}

		result = mt[mti++];

		result ^= (result >> TEMPERING_SHIFT_U);
		result ^= (result << TEMPERING_SHIFT_S) & TEMPERING_MASK_B;
		result ^= (result << TEMPERING_SHIFT_T) & TEMPERING_MASK_C;
		result ^= (result >> TEMPERING_SHIFT_L);

		return result;
	}

	/**
	* Fast scalar parameter setter structure for a material.
	***********************************************************************************/

	struct FMaterialScalarParameterSetter
	{
	public:

		void Setup(UMaterialInstanceDynamic* material, const FName& parameterName, float defaultValue)
		{
			Material = material;
			CurrentValue = defaultValue;

			Material->InitializeScalarParameterAndGetIndex(parameterName, defaultValue, ParameterIndex);
		}

		void Setup(UMaterialInstanceDynamic* material, const FName& parameterName)
		{ Setup(material, parameterName, CurrentValue); }

		UMaterialInstanceDynamic* Material = nullptr;

		int32 ParameterIndex = INDEX_NONE;

		float CurrentValue = 0.0f;

		void Set(float newValue)
		{
			if (CurrentValue != newValue)
			{
				CurrentValue = newValue;

				if (Material != nullptr &&
					ParameterIndex != INDEX_NONE)
				{
					Material->SetScalarParameterByIndex(ParameterIndex, newValue);
				}
			}
		}
	};

	/**
	* Fast vector parameter setter structure for a material.
	***********************************************************************************/

	struct FMaterialVectorParameterSetter
	{
	public:

		void Setup(UMaterialInstanceDynamic* material, const FName& parameterName, const FLinearColor& defaultValue)
		{
			Material = material;
			CurrentValue = defaultValue;

			Material->InitializeVectorParameterAndGetIndex(parameterName, defaultValue, ParameterIndex);
		}

		void Setup(UMaterialInstanceDynamic* material, const FName& parameterName)
		{ Setup(material, parameterName, CurrentValue); }

		UMaterialInstanceDynamic* Material = nullptr;

		int32 ParameterIndex = INDEX_NONE;

		FLinearColor CurrentValue = FLinearColor::Transparent;

		void Set(const FLinearColor& newValue)
		{
			if (CurrentValue != newValue)
			{
				CurrentValue = newValue;

				if (Material != nullptr &&
					ParameterIndex != INDEX_NONE)
				{
					Material->SetVectorParameterByIndex(ParameterIndex, newValue);
				}
			}
		}
	};

	/**
	* Outcodes and a rectangle structure for clipping.
	***********************************************************************************/

	using OutCode = int32;

	const OutCode OutCodeInside = 0;
	const OutCode OutCodeLeft = 1;
	const OutCode OutCodeRight = 2;
	const OutCode OutCodeBottom = 4;
	const OutCode OutCodeTop = 8;

	struct FRectangle
	{
		FVector2D Min;
		FVector2D Max;
	};

	/**
	* Compute the bit code for a point (x, y) using the clip rectangle bounded
	* diagonally by rectangle.
	***********************************************************************************/

	inline OutCode ComputeOutCode(const FVector2D& xy, const FRectangle& rectangle)
	{
		OutCode code = OutCodeInside;

		if (xy.X < rectangle.Min.X)           // to the left of clip window
			code |= OutCodeLeft;
		else if (xy.X > rectangle.Max.X)      // to the right of clip window
			code |= OutCodeRight;
		if (xy.Y < rectangle.Min.Y)           // below the clip window
			code |= OutCodeBottom;
		else if (xy.Y > rectangle.Max.Y)      // above the clip window
			code |= OutCodeTop;

		return code;
	}

	/**
	* Cohen–Sutherland clipping algorithm clips a line from xy0 to xy1 against a
	* rectangle.
	***********************************************************************************/

	inline void CohenSutherlandLineClip(FVector2D& xy0, FVector2D& xy1, const FRectangle& rectangle)
	{
		OutCode outcode0 = ComputeOutCode(xy0, rectangle);
		OutCode outcode1 = ComputeOutCode(xy1, rectangle);

		while (true)
		{
			if (!(outcode0 | outcode1))
			{
				// Bitwise OR is 0. Trivially accept and get out of loop.

				break;
			}
			else if (outcode0 & outcode1)
			{
				// Bitwise AND is not 0. (implies both end points are in the same region outside the window).
				// Reject and get out of loop.

				break;
			}
			else
			{
				// Failed both tests, so calculate the line segment to clip
				// from an outside point to an intersection with clip edge.

				float x = 0.0f, y = 0.0f;

				// At least one endpoint is outside the clip rectangle; pick it.

				OutCode outcodeOut = outcode0 ? outcode0 : outcode1;

				// Now find the intersection point;
				// use formulas y = y0 + slope * (x - x0), x = x0 + (1 / slope) * (y - y0)

				if (outcodeOut & OutCodeTop)
				{
					// Point is above the clip rectangle.

					x = xy0.X + (xy1.X - xy0.X) * (rectangle.Max.Y - xy0.Y) / (xy1.Y - xy0.Y);
					y = rectangle.Max.Y;
				}
				else if (outcodeOut & OutCodeBottom)
				{
					// Point is below the clip rectangle.

					x = xy0.X + (xy1.X - xy0.X) * (rectangle.Min.Y - xy0.Y) / (xy1.Y - xy0.Y);
					y = rectangle.Min.Y;
				}
				else if (outcodeOut & OutCodeRight)
				{
					// Point is to the right of clip rectangle.

					y = xy0.Y + (xy1.Y - xy0.Y) * (rectangle.Max.X - xy0.X) / (xy1.X - xy0.X);
					x = rectangle.Max.X;
				}
				else if (outcodeOut & OutCodeLeft)
				{
					// Point is to the left of clip rectangle.

					y = xy0.Y + (xy1.Y - xy0.Y) * (rectangle.Min.X - xy0.X) / (xy1.X - xy0.X);
					x = rectangle.Min.X;
				}

				// Now we move outside point to intersection point to clip
				// and get ready for next pass.

				if (outcodeOut == outcode0)
				{
					xy0.X = x;
					xy0.Y = y;
					outcode0 = ComputeOutCode(xy0, rectangle);
				}
				else
				{
					xy1.X = x;
					xy1.Y = y;
					outcode1 = ComputeOutCode(xy1, rectangle);
				}
			}
		}
	}

	// namespace FMathEx
}

#include "perlinnoise.h"
