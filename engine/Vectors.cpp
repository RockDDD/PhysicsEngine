#include "Common.h"
#include "Vectors.h"

float CorrectDegrees(float degrees)
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
float RAD2DEG(float radians)
{
	float degrees = radians * 57.295754f;
	degrees = CorrectDegrees(degrees);
	return degrees;
}
#endif

#ifndef DEG2RAD
float DEG2RAD(float degrees)
{
	degrees = CorrectDegrees(degrees);
	float radians = degrees * 0.0174533f;
	return radians;
}
#endif

bool operator==(const Vec2& l, const Vec2& r)
{
	return CMP(l.x, r.x) && CMP(l.y, r.y);
}

bool operator==(const Vec3& l, const Vec3& r)
{
	return CMP(l.x, r.x) && CMP(l.y, r.y) && CMP(l.z, r.z);
}

bool operator!=(const Vec2& l, const Vec2& r)
{
	return !(l == r);
}

bool operator!=(const Vec3& l, const Vec3& r)
{
	return !(l == r);
}

Vec2 operator+(const Vec2& l, const Vec2& r)
{
	return { l.x + r.x, l.y + r.y };
}

Vec3 operator+(const Vec3& l, const Vec3& r)
{
	return { l.x + r.x, l.y + r.y, l.z + r.z };
}

Vec2 operator-(const Vec2& l, const Vec2& r)
{
	return { l.x - r.x, l.y - r.y };
}

Vec3 operator-(const Vec3& l, const Vec3& r)
{
	return { l.x - r.x, l.y - r.y, l.z - r.z };
}

Vec2 operator*(const Vec2& l, const Vec2& r)
{
	return { l.x * r.x, l.y * r.y };
}

Vec3 operator*(const Vec3& l, const Vec3& r)
{
	return { l.x * r.x, l.y * r.y, l.z * r.z };
}

Vec2 operator*(const Vec2& l, float r)
{
	return { l.x * r, l.y * r };
}

Vec3 operator*(const Vec3& l, float r)
{
	return { l.x * r, l.y * r, l.z * r };
}

Vec2 operator/(const Vec2& l, const Vec2& r)
{
	return{ l.x / r.x, l.y / r.y };
}

Vec3 operator/(const Vec3& l, const Vec3& r)
{
	return{ l.x / r.x, l.y / r.y, l.z / r.z };
}

Vec2 operator/(const Vec2& l, float r)
{
	return{ l.x / r, l.y / r };
}

Vec3 operator/(const Vec3& l, float r)
{
	return{ l.x / r, l.y / r, l.z / r };
}

std::ostream& operator<<(std::ostream& os, const Vec2& m)
{
	os << "(" << m.x << ", " << m.y << ")";
	return os;
}

std::ostream& operator<<(std::ostream& os, const Vec3& m)
{
	os << "(" << m.x << ", " << m.y << ", " << m.z << ")";
	return os;
}

float Dot(const Vec2& l, const Vec2& r)
{
	return l.x * r.x + l.y * r.y;
}

float Dot(const Vec3& l, const Vec3& r)
{
	return l.x * r.x + l.y * r.y + l.z * r.z;
}

Vec2& operator+=(Vec2& l, const Vec2& r)
{
	l.x += r.x;
	l.y += r.y;
	return l;
}

Vec2& operator-=(Vec2& l, const Vec2& r)
{
	l.x -= r.y;
	l.y -= r.y;
	return l;
}

Vec2& operator*=(Vec2& l, const Vec2& r)
{
	l.x *= r.x;
	l.y *= r.y;
	return l;
}

Vec2& operator*=(Vec2& l, const float r)
{
	l.x *= r;
	l.y *= r;
	return l;
}

Vec2& operator/=(Vec2& l, const Vec2& r)
{
	l.x /= r.x;
	l.y /= r.y;
	return l;
}

Vec2& operator/=(Vec2& l, const float r)
{
	l.x /= r;
	l.y /= r;
	return l;
}

Vec3& operator+=(Vec3& l, const Vec3& r)
{
	l.x += r.x;
	l.y += r.y;
	l.z += r.z;
	return l;
}

Vec3& operator-=(Vec3& l, const Vec3& r)
{
	l.x -= r.x;
	l.y -= r.y;
	l.z -= r.z;
	return l;
}

Vec3& operator*=(Vec3& l, const Vec3& r) {
	l.x *= r.x;
	l.y *= r.y;
	l.z *= r.z;
	return l;
}

Vec3& operator*=(Vec3& l, const float r)
{
	l.x *= r;
	l.y *= r;
	l.z *= r;
	return l;
}

Vec3& operator/=(Vec3& l, const Vec3& r)
{
	l.x /= r.x;
	l.y /= r.y;
	l.z /= r.z;
	return l;
}

Vec3& operator/=(Vec3& l, const float r)
{
	l.x /= r;
	l.y /= r;
	l.z /= r;
	return l;
}

float Magnitude(const Vec2& v)
{
	return sqrtf(Dot(v, v));
}

float Magnitude(const Vec3& v)
{
	return sqrtf(Dot(v, v));
}

float MagnitudeSq(const Vec2& v)
{
	return Dot(v, v);
}

float MagnitudeSq(const Vec3& v)
{
	return Dot(v, v);
}

float Distance(const Vec2& p1, const Vec2& p2)
{
	return Magnitude(p1 - p2);
}

float Distance(const Vec3& p1, const Vec3& p2)
{
	return Magnitude(p1 - p2);
}

float DistanceSq(const Vec2& p1, const Vec2& p2)
{
	return MagnitudeSq(p1 - p2);
}

float DistanceSq(const Vec3& p1, const Vec3& p2)
{
	return MagnitudeSq(p1 - p2);
}

Vec2 RotateVector(const Vec2& vector, float degrees)
{
	degrees = DEG2RAD(degrees);
	float s = sinf(degrees);
	float c = cosf(degrees);

	return Vec2(vector.x * c - vector.y * s, vector.x * s + vector.y * c);
}

void Normalize(Vec2& v)
{
	v = v * (1.0f / Magnitude(v));
}

void Normalize(Vec3& v)
{
	v = v * (1.0f / Magnitude(v));
}

Vec2 Normalized(const Vec2& v)
{
	return v * (1.0f / Magnitude(v));
}
Vec3 Normalized(const Vec3& v)
{
	return v * (1.0f / Magnitude(v));
}

Vec3 Cross(const Vec3& l, const Vec3& r)
{
	Vec3 result;
	result.x = l.y * r.z - l.z * r.y;
	result.y = l.z * r.x - l.x * r.z;
	result.z = l.x * r.y - l.y * r.x;
	return result;
}

float Angle(const Vec2& l, const Vec2& r)
{
	return acosf(Dot(l, r) / sqrtf(MagnitudeSq(l) * MagnitudeSq(r)));
}

float Angle(const Vec3& l, const Vec3& r)
{
	return acosf(Dot(l, r) / sqrtf(MagnitudeSq(l) * MagnitudeSq(r)));
}

Vec2 Project(const Vec2& length, const Vec2& direction)
{
	float dot = Dot(length, direction);
	float magSq = MagnitudeSq(direction);
	return direction * (dot / magSq);
}

Vec3 Project(const Vec3& length, const Vec3& direction)
{
	float dot = Dot(length, direction);
	float magSq = MagnitudeSq(direction);
	return direction * (dot / magSq);
}

Vec2 Perpendicular(const Vec2& length, const Vec2& direction)
{
	return length - Project(length, direction);
}

Vec3 Perpendicular(const Vec3& length, const Vec3& direction)
{
	return length - Project(length, direction);
}

Vec2 Reflection(const Vec2& sourceVector, const Vec2& normal)
{
	return sourceVector - normal * (Dot(sourceVector, normal) * 2.0f);
}

Vec3 Reflection(const Vec3& sourceVector, const Vec3& normal)
{
	return sourceVector - normal * (Dot(sourceVector, normal) * 2.0f);
}