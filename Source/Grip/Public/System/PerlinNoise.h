/**
*
* Perlin Noise class.
*
* Original author: Rob Baker.
* Current maintainer: Rob Baker.
*
* Copyright Caged Element Inc, code provided for educational purposes only.
*
* An implementation of the Perlin noise technique, over 1, 2 or 3 dimensions.
*
***********************************************************************************/

#pragma once

#include "system/mathhelpers.h"

/**
* Perlin noise.
***********************************************************************************/

struct FPerlinNoise
{
public:

	// Construct the Perlin object with a seed for the noise.
	FPerlinNoise()
	{ Initialize(0xcafebabe); }

	// Initialize the Perlin object with a seed for the noise.
	void Initialize(uint32 seed);

	// Get some one dimensional noise.
	float Noise1(float x) const;

	// Get some two dimensional noise.
	float Noise2(float x, float y) const;

	// Get some three dimensional noise.
	float Noise3(float x, float y, float z) const;

	// Get the random number generator used for creating noise.
	FMathEx::FRandomFast& GetRandom()
	{ return Random; }

private:

	static const int32 SIZE = 256;
	static const int32 MASK = SIZE - 1;

	// Permutation table
	uint8 p[SIZE];

	// Gradients
	float gx[SIZE];
	float gy[SIZE];
	float gz[SIZE];

	FMathEx::FRandomFast Random;
};

namespace FMathEx
{

	/**
	* Update an oscillator for movement noise.
	***********************************************************************************/

	inline float UpdateOscillator(FMathEx::FOscillator& oscillator, FPerlinNoise& noise, float deltaSeconds)
	{
		if (oscillator.Amplitude != 0.0f)
		{
			oscillator.Tick(deltaSeconds);

			float height = noise.Noise1(oscillator.Time * 0.2f) * 10.0f;

			return oscillator.Amplitude * height;
		}

		return 0.0f;
	}

	// namespace FMathEx
}
