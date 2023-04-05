#pragma once
#include <cmath>
#include <cfloat>
#include <stdint.h>

inline float CorrectDegrees(float degrees)
{
	while (degrees > 360.0f)
	{
		degrees -= 360.0f;
	}
	while (degrees < -360.0f)
	{
		degrees += 360.0f;
	}
	return degrees;
}

#ifndef RAD2DEG
inline float RAD2DEG(float radians)
{
	float degrees = radians * 57.295754f;
	degrees = CorrectDegrees(degrees);
	return degrees;
}
#endif

#ifndef DEG2RAD
inline float DEG2RAD(float degrees)
{
	degrees = CorrectDegrees(degrees);
	float radians = degrees * 0.0174533f;
	return radians;
}
#endif

inline bool AlmostEqualRelativeAndAbs(float A, float B, float maxDiff, float maxRelDiff = FLT_EPSILON)
{
	// Check if the numbers are really close -- needed when comparing numbers near zero.
	float diff = fabs(A - B);
	if (diff <= maxDiff)
	{
		return true;
	}

	A = fabs(A);
	B = fabs(B);
	float largest = (B > A) ? B : A;

	if (diff <= largest * maxRelDiff)
	{
		return true;
	}
	return false;
}

#define CMP(x, y) AlmostEqualRelativeAndAbs(x, y, 0.005f)