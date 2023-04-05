#pragma once
#include <cmath>
#include <cfloat>
#include <stdint.h>

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