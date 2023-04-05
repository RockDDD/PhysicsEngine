#pragma once

#include <ostream>

struct Vec2
{
	union
	{
		struct
		{
			float x;
			float y;
		};
		float asArray[2];
	};
	inline float& operator[](int i)
	{
		return asArray[i];
	}

	inline Vec2() :x(0.0f), y(0.0f) {}

	inline Vec2(float x, float y) : x(x), y(y) {}
};

struct Vec3
{
	union
	{
		struct
		{
			float x;
			float y;
			float z;
		};
		float asArray[3];
	};
	inline float& operator[](int i)
	{
		return asArray[i];
	}

	inline Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
	inline Vec3(float x, float y, float z) : x(x), y(y), z(z) {}
};

Vec2 operator+(const Vec2& l, const Vec2& r);
Vec3 operator+(const Vec3& l, const Vec3& r);

Vec2 operator-(const Vec2& l, const Vec2& r);
Vec3 operator-(const Vec3& l, const Vec3& r);

Vec2 operator*(const Vec2& l, const Vec2& r);
Vec3 operator*(const Vec3& l, const Vec3& r);

Vec2 operator*(const Vec2& l, float r);
Vec3 operator*(const Vec3& l, float r);

Vec2 operator/(const Vec2& l, const Vec2& r);
Vec3 operator/(const Vec3& l, const Vec3& r);

Vec2 operator/(const Vec2& l, float r);
Vec3 operator/(const Vec3& l, float r);

std::ostream& operator<<(std::ostream& os, const Vec2& m);
std::ostream& operator<<(std::ostream& os, const Vec3& m);

bool operator==(const Vec2& l, const Vec2& r);
bool operator==(const Vec3& l, const Vec3& r);

bool operator!=(const Vec2& l, const Vec2& r);
bool operator!=(const Vec3& l, const Vec3& r);

Vec2& operator+=(Vec2& l, const Vec2& r);
Vec2& operator-=(Vec2& l, const Vec2& r);
Vec2& operator*=(Vec2& l, const Vec2& r);
Vec2& operator*=(Vec2& l, const float r);
Vec2& operator/=(Vec2& l, const Vec2& r);
Vec2& operator/=(Vec2& l, const float r);

Vec3& operator+=(Vec3& l, const Vec3& r);
Vec3& operator-=(Vec3& l, const Vec3& r);
Vec3& operator*=(Vec3& l, const Vec3& r);
Vec3& operator*=(Vec3& l, const float r);
Vec3& operator/=(Vec3& l, const Vec3& r);
Vec3& operator/=(Vec3& l, const float r);

float Dot(const Vec2& l, const Vec2& r);
float Dot(const Vec3& l, const Vec3& r);

float Magnitude(const Vec2& v);
float Magnitude(const Vec3& v);

float MagnitudeSq(const Vec2& v);
float MagnitudeSq(const Vec3& v);

float Distance(const Vec2& p1, const Vec2& p2);
float Distance(const Vec3& p1, const Vec3& p2);

float DistanceSq(const Vec2& p1, const Vec2& p2);
float DistanceSq(const Vec3& p1, const Vec3& p2);

Vec2 RotateVector(const Vec2& vector, float degrees);

void Normalize(Vec2& v);

void Normalize(Vec3& v);

Vec2 Normalized(const Vec2& v);
Vec3 Normalized(const Vec3& v);

Vec3 Cross(const Vec3& l, const Vec3& r);

float Angle(const Vec2& l, const Vec2& r);
float Angle(const Vec3& l, const Vec3& r);

Vec2 Project(const Vec2& length, const Vec2& direction);
Vec3 Project(const Vec3& length, const Vec3& direction);

Vec2 Perpendicular(const Vec2& length, const Vec2& direction);
Vec3 Perpendicular(const Vec3& length, const Vec3& direction);

Vec2 Reflection(const Vec2& sourceVector, const Vec2& normal);
Vec3 Reflection(const Vec3& sourceVector, const Vec3& normal);