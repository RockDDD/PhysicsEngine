#include "Mat.h"
#include "Common.h"
#include "Geometry2D.h"

#define CLAMP(number, minimum, maximum) \
	number = (number < minimum) ? minimum : \
		((number > maximum) ? maximum : number)

#define OVERLAP(minA, minB, maxA, maxB) ((minB <= maxA) && (minA <= maxB))

float Length(const Line2D& line)
{
	return Magnitude(line.end - line.start);
}

float LengthSq(const Line2D& line)
{
	return MagnitudeSq(line.end - line.start);
}

Vec2 GetMin(const Rectangle2D& rect)
{
	Vec2 p1 = rect.origin;
	Vec2 p2 = rect.origin + rect.size;

	return Vec2(fminf(p1.x, p2.x), fminf(p1.y, p2.y));
}

Vec2 GetMax(const Rectangle2D& rect)
{
	Vec2 p1 = rect.origin;
	Vec2 p2 = rect.origin + rect.size;

	return Vec2(fmaxf(p1.x, p2.x), fmaxf(p1.y, p2.y));
}

Rectangle2D FromMinMax(const Vec2& min, const Vec2& max)
{
	return Rectangle2D(min, max - min);
}

bool PointOnLine(const Point2D& point, const Line2D& line)
{
	// Find the slope
	float M = (line.end.y - line.start.y) / (line.end.x - line.start.x);
	// Find the Y-Intercept
	float B = line.start.y - M * line.start.x;
	// Check line equation
	return CMP(point.y, M * point.x + B);
}

bool PointInCircle(const Point2D& point, const Circle& circle)
{
	Line2D line(point, circle.position);
	if (LengthSq(line) < circle.radius * circle.radius)
	{
		return true;
	}
	return false;
}

bool PointInRectangle(const Point2D& point, const Rectangle2D& rectangle)
{
	Vec2 min = GetMin(rectangle);
	Vec2 max = GetMax(rectangle);

	return	min.x <= point.x && min.y <= point.y &&
		point.x <= max.x && point.y <= max.y;
}

bool PointInOrientedRectangle(const Point2D& point, const OrientedRectangle& rectangle)
{
	Rectangle2D localRectangle(Point2D(), rectangle.halfExtents * 2.0f);
	Vec2 rotVector = point - rectangle.position;
	float theta = -DEG2RAD(rectangle.rotation);
	float zRotation2x2[] = { // Construct matrix
		cosf(theta), sinf(theta),
		-sinf(theta), cosf(theta) };
	// Rotate vector
	Multiply(rotVector.asArray, Vec2(rotVector.x, rotVector.y).asArray, 1, 2, zRotation2x2, 2, 2);
	Vec2 localPoint = rotVector + rectangle.halfExtents;
	return PointInRectangle(localPoint, localRectangle);
}

std::ostream& operator<<(std::ostream& os, const Line2D& shape)
{
	os << "start: (" << shape.start.x << ", " << shape.start.y << "), end: (" << shape.end.x << ", " << shape.end.y << ")";
	return os;
}

std::ostream& operator<<(std::ostream& os, const Circle& shape)
{
	os << "position: (" << shape.position.x << ", " << shape.position.y << "), radius: " << shape.radius;
	return os;
}
std::ostream& operator<<(std::ostream& os, const Rectangle2D& shape)
{
	Vec2 min = GetMin(shape);
	Vec2 max = GetMax(shape);

	os << "min: (" << min.x << ", " << min.y << "), max: (" << max.x << ", " << max.y << ")";
	return os;
}

std::ostream& operator<<(std::ostream& os, const OrientedRectangle& shape)
{
	os << "position: (" << shape.position.x << ", " << shape.position.y << "), half size: (";
	os << shape.halfExtents.x << ", " << shape.halfExtents.y << "), rotation: " << shape.rotation;
	return os;
}

bool LineCircle(const Line2D& line, const Circle& circle)
{
	// Turn line into vector
	Vec2 ab = line.end - line.start;

	// Project point (circle position) onto ab (line segment), computing the 
	// paramaterized position d(t) = a + t * (b - a)
	float t = Dot(circle.position - line.start, ab) / Dot(ab, ab);

	// Find the closest point on the line segment
	Point2D closestPoint = line.start + ab * t;

	Line2D circleToClosest(circle.position, closestPoint);
	return LengthSq(circleToClosest) < circle.radius * circle.radius;
}

bool LineRectangle(const Line2D& line, const Rectangle2D& rect)
{
	if (PointInRectangle(line.start, rect) || PointInRectangle(line.end, rect))
	{
		return true;
	}

	Vec2 norm = Normalized(line.end - line.start);
	norm.x = (norm.x != 0) ? 1.0f / norm.x : 0;
	norm.y = (norm.y != 0) ? 1.0f / norm.y : 0;
	Vec2 min = (GetMin(rect) - line.start) * norm;
	Vec2 max = (GetMax(rect) - line.start) * norm;

	float tmin = fmaxf(fminf(min.x, max.x), fminf(min.y, max.y));
	float tmax = fminf(fmaxf(min.x, max.x), fmaxf(min.y, max.y));
	if (tmax < 0 || tmin > tmax)
	{
		return false;
	}
	float t = (tmin < 0.0f) ? tmax : tmin;
	return t > 0.0f && t * t < LengthSq(line);
}

bool LineOrientedRectangle(const Line2D& line, const OrientedRectangle& rectangle)
{
	float theta = -DEG2RAD(rectangle.rotation);
	float zRotation2x2[] = {
		cosf(theta), sinf(theta),
		-sinf(theta), cosf(theta) };
	Line2D localLine;

	Vec2 rotVector = line.start - rectangle.position;
	Multiply(rotVector.asArray, Vec2(rotVector.x, rotVector.y).asArray, 1, 2, zRotation2x2, 2, 2);
	localLine.start = rotVector + rectangle.halfExtents;

	rotVector = line.end - rectangle.position;
	Multiply(rotVector.asArray, Vec2(rotVector.x, rotVector.y).asArray, 1, 2, zRotation2x2, 2, 2);
	localLine.end = rotVector + rectangle.halfExtents;

	Rectangle2D localRectangle(Point2D(), rectangle.halfExtents * 2.0f);
	return LineRectangle(localLine, localRectangle);
}

bool CircleCircle(const Circle& circle1, const Circle& circle2)
{
	Line2D line(circle1.position, circle2.position);
	float radiiSum = circle1.radius + circle2.radius;
	return LengthSq(line) <= radiiSum * radiiSum;
}

bool CircleRectangle(const Circle& circle, const Rectangle2D& rect)
{
	Point2D closestPoint = circle.position;
	Vec2 min = GetMin(rect);
	Vec2 max = GetMax(rect);
	closestPoint.x = (closestPoint.x < min.x) ? min.x :
		(closestPoint.x > max.x) ? max.x : closestPoint.x;
	closestPoint.y = (closestPoint.y < min.y) ? min.y :
		(closestPoint.y > max.y) ? max.y : closestPoint.y;

	Line2D line(circle.position, closestPoint);
	return LengthSq(line) <= circle.radius * circle.radius;
}

bool CircleOrientedRectangle(const Circle& circle, const OrientedRectangle& rect)
{
	Vec2 rotVector = circle.position - rect.position;
	float theta = -DEG2RAD(rect.rotation);
	float zRotation2x2[] = {
		cosf(theta), sinf(theta),
		-sinf(theta), cosf(theta) };
	Multiply(rotVector.asArray, Vec2(rotVector.x, rotVector.y).asArray, 1, 2, zRotation2x2, 2, 2);

	Circle localCircle(rotVector + rect.halfExtents, circle.radius);
	Rectangle2D localRectangle(Point2D(), rect.halfExtents * 2.0f);

	return CircleRectangle(localCircle, localRectangle);
}

bool RectangleRectangle(const Rectangle2D& rect1, const Rectangle2D& rect2)
{
	Vec2 aMin = GetMin(rect1);
	Vec2 aMax = GetMax(rect1);
	Vec2 bMin = GetMin(rect2);
	Vec2 bMax = GetMax(rect2);

	bool xOverlap = ((bMin.x <= aMax.x) && (aMin.x <= bMax.x));
	bool yOverlap = ((bMin.y <= aMax.y) && (aMin.y <= bMax.y));

	return xOverlap && yOverlap;
}

Interval2D GetInterval(const Rectangle2D& rect, const Vec2& axis)
{
	Vec2 min = GetMin(rect);
	Vec2 max = GetMax(rect);

	Vec2 verts[] = { // Get all vertices of rect
		Vec2(min.x, min.y),
		Vec2(min.x, max.y),
		Vec2(max.x, max.y),
		Vec2(max.x, min.y)
	};

	// Set interval first projected vertex
	Interval2D result;
	result.min = Dot(axis, verts[0]);
	result.max = result.min;

	// For all other verts
	for (int i = 1; i < 4; ++i)
	{
		// Project vertex
		float projection = Dot(axis, verts[i]);
		result.min = (projection < result.min) ? projection : result.min;
		result.max = (projection > result.max) ? projection : result.max;
	}

	return result;
}

bool OverlapOnAxis(const Rectangle2D& rect1, const Rectangle2D& rect2, const Vec2& axis)
{
	Interval2D a = GetInterval(rect1, axis);
	Interval2D b = GetInterval(rect2, axis);
	return ((b.min <= a.max) && (a.min <= b.max));
}

bool RectangleRectangleSAT(const Rectangle2D& rect1, const Rectangle2D& rect2)
{
	Vec2 axisToTest[] =
	{
		Vec2(1, 0),
		Vec2(0, 1)
	};

	for (int i = 0; i < 2; ++i)
	{
		if (!OverlapOnAxis(rect1, rect2, axisToTest[i]))
		{
			// Intervals don't overlap, seperating axis found
			return false; // No collision has taken place
		}
	}

	// All intervals overlapped, seperating axis not found
	return true; // We have a collision
}

Interval2D GetInterval(const OrientedRectangle& rect, const Vec2& axis)
{
	Rectangle2D nonOrientedRect = Rectangle2D(Point2D(rect.position - rect.halfExtents), rect.halfExtents * 2.0f);
	Vec2 min = GetMin(nonOrientedRect);
	Vec2 max = GetMax(nonOrientedRect);
	Vec2 verts[] = { min, max, Vec2(min.x, max.y), Vec2(max.x, min.y) };

	float theta = DEG2RAD(rect.rotation);
	float zRotation2x2[] =
	{
		cosf(theta), sinf(theta),
		-sinf(theta), cosf(theta) };

	for (int i = 0; i < 4; ++i)
	{
		Vec2 rotVector = verts[i] - rect.position;
		Multiply(rotVector.asArray, Vec2(rotVector.x, rotVector.y).asArray, 1, 2, zRotation2x2, 2, 2);
		verts[i] = rotVector + rect.position;
	}

	// Set interval first projected vertex
	Interval2D result;
	result.min = result.max = Dot(axis, verts[0]);

	// For all other verts
	for (int i = 1; i < 4; ++i)
	{
		// Project vertex
		float projection = Dot(axis, verts[i]);
		result.min = (projection < result.min) ? projection : result.min;
		result.max = (projection > result.max) ? projection : result.max;
	}

	return result;
}

bool OverlapOnAxis(const Rectangle2D& rect1, const OrientedRectangle& rect2, const Vec2& axis)
{
	Interval2D a = GetInterval(rect1, axis);
	Interval2D b = GetInterval(rect2, axis);
	return ((b.min <= a.max) && (a.min <= b.max));
}

bool RectangleOrientedRectangle(const Rectangle2D& rect1, const OrientedRectangle& rect2)
{
	Vec2 axisToTest[]
	{
		Vec2(1, 0),
		Vec2(0, 1),
		Normalized(Vec2(rect2.halfExtents.x, 0)),
		Normalized(Vec2(0, rect2.halfExtents.y))
	};

	float theta = DEG2RAD(rect2.rotation);
	float zRotation2x2[] =
	{
		cosf(theta), sinf(theta),
		-sinf(theta), cosf(theta)
	};
	Multiply(axisToTest[2].asArray, Normalized(Vec2(rect2.halfExtents.x, 0)).asArray, 1, 2, zRotation2x2, 2, 2);
	Multiply(axisToTest[3].asArray, Normalized(Vec2(0, rect2.halfExtents.y)).asArray, 1, 2, zRotation2x2, 2, 2);

	for (int i = 0; i < 4; ++i)
	{
		if (!OverlapOnAxis(rect1, rect2, axisToTest[i]))
		{
			// Intervals don't overlap, seperating axis found
			return false; // No collision has taken place
		}
	}

	// All intervals overlapped, seperating axis not found
	return true; // We have a collision
}

bool OverlapOnAxis(const OrientedRectangle& rect1, const OrientedRectangle& rect2, const Vec2& axis)
{
	Interval2D a = GetInterval(rect1, axis);
	Interval2D b = GetInterval(rect2, axis);
	return ((b.min <= a.max) && (a.min <= b.max));
}

bool OrientedRectangleOrientedRectangleSAT(const OrientedRectangle& rect1, const OrientedRectangle& rect2)
{
	Vec2 axisToTest[]
	{
		Vec2(1, 0),
		Vec2(0, 1),
		Vec2(), // TEMP
		Vec2(), // TEMP
		Vec2(), // TEMP
		Vec2() // TEMP
	};

	// Collision axis of rect 2
	float theta = DEG2RAD(rect2.rotation);
	float zRotation2x2[] =
	{
		cosf(theta), sinf(theta),
		-sinf(theta), cosf(theta)
	};
	Multiply(axisToTest[2].asArray, Normalized(Vec2(rect2.halfExtents.x, 0)).asArray, 1, 2, zRotation2x2, 2, 2);
	Multiply(axisToTest[3].asArray, Normalized(Vec2(0, rect2.halfExtents.y)).asArray, 1, 2, zRotation2x2, 2, 2);

	// Collision axis of rect 1
	theta = DEG2RAD(rect1.rotation);
	zRotation2x2[0] = cosf(theta);
	zRotation2x2[1] = sinf(theta);
	zRotation2x2[2] = -sinf(theta);
	zRotation2x2[3] = cosf(theta);
	Multiply(axisToTest[4].asArray, Normalized(Vec2(rect1.halfExtents.x, 0)).asArray, 1, 2, zRotation2x2, 2, 2);
	Multiply(axisToTest[5].asArray, Normalized(Vec2(0, rect1.halfExtents.y)).asArray, 1, 2, zRotation2x2, 2, 2);

	for (int i = 0; i < 6; ++i)
	{
		if (!OverlapOnAxis(rect1, rect2, axisToTest[i]))
		{
			// Intervals don't overlap, seperating axis found
			return false; // No collision has taken place
		}
	}

	// All intervals overlapped, seperating axis not found
	return true; // We have a collision
}

bool OrientedRectangleOrientedRectangle(const OrientedRectangle& r1, const OrientedRectangle& r2)
{
	//return OrientedRectangleOrientedRectangleSAT(rect1, rect2);
	Rectangle2D localRect1(Point2D(), r1.halfExtents * 2.0f);
	OrientedRectangle localRect2(r2.position, r2.halfExtents, r2.rotation);

	localRect2.rotation = r2.rotation - r1.rotation;
	Vec2 rotVector = r2.position - r1.position;
	float theta = -DEG2RAD(r1.rotation);
	float zRotation2x2[] =
	{
		cosf(theta), sinf(theta),
		-sinf(theta), cosf(theta)
	};
	Multiply(rotVector.asArray, Vec2(rotVector.x, rotVector.y).asArray, 1, 2, zRotation2x2, 2, 2);
	localRect2.position = rotVector + r1.halfExtents;

	return RectangleOrientedRectangle(localRect1, localRect2);
}

Circle ContainingCircle(Point2D* pointArray, int arrayCount)
{
	Point2D center;
	for (int i = 0; i < arrayCount; ++i)
	{
		center = center + pointArray[i];
	}
	center = center * (1.0f / (float)arrayCount);

	Circle result(center, 1.0f);

	// Find the squared radius
	result.radius = MagnitudeSq(center - pointArray[0]);
	for (int i = 1; i < arrayCount; ++i)
	{
		float distance = MagnitudeSq(center - pointArray[i]);
		if (distance > result.radius)
		{
			result.radius = distance;
		}
	}

	// This has been squared until now
	result.radius = sqrtf(result.radius);
	return result;
}

Circle ContainingCircleAlt(Point2D* pointArray, int arrayCount)
{
	Vec2 min = pointArray[0];
	Vec2 max = pointArray[0];

	for (int i = 0; i < arrayCount; ++i)
	{
		min.x = pointArray[i].x < min.x ? pointArray[i].x : min.x;
		min.y = pointArray[i].y < min.y ? pointArray[i].y : min.y;
		max.x = pointArray[i].x > max.x ? pointArray[i].x : max.x;
		max.y = pointArray[i].y > max.y ? pointArray[i].y : max.y;
	}

	return Circle((min + max) * 0.5f, Magnitude(max - min) * 0.5f);
}

Rectangle2D ContainingRectangle(Point2D* pointArray, int arrayCount)
{
	Vec2 min = pointArray[0];
	Vec2 max = pointArray[0];

	for (int i = 0; i < arrayCount; ++i) {
		min.x = pointArray[i].x < min.x ? pointArray[i].x : min.x;
		min.y = pointArray[i].y < min.y ? pointArray[i].y : min.y;
		max.x = pointArray[i].x > max.x ? pointArray[i].x : max.x;
		max.y = pointArray[i].y > max.y ? pointArray[i].y : max.y;
	}

	return FromMinMax(min, max);
}

bool PointInShape(const BoundingShape& shape, const Point2D& point)
{
	for (int i = 0; i < shape.numCircles; ++i)
	{
		if (PointInCircle(point, shape.circles[i]))
		{
			return true;
		}
	}
	for (int i = 0; i < shape.numRectangles; ++i)
	{
		if (PointInRectangle(point, shape.rectangles[i]))
		{
			return true;
		}
	}
	return false;
}

bool LineShape(const Line2D& line, const BoundingShape& shape) {
	for (int i = 0; i < shape.numCircles; ++i) {
		if (LineCircle(line, shape.circles[i])) {
			return true;
		}
	}
	for (int i = 0; i < shape.numRectangles; ++i) {
		if (LineRectangle(line, shape.rectangles[i])) {
			return true;
		}
	}
	return false;
}

bool CircleShape(const Circle& circle, const BoundingShape& shape) {
	for (int i = 0; i < shape.numCircles; ++i) {
		if (CircleCircle(circle, shape.circles[i])) {
			return true;
		}
	}
	for (int i = 0; i < shape.numRectangles; ++i) {
		if (CircleRectangle(circle, shape.rectangles[i])) {
			return true;
		}
	}
	return false;
}

bool RectangleShape(const Rectangle2D& rectangle, const BoundingShape& shape)
{
	for (int i = 0; i < shape.numCircles; ++i) 
	{
		if (RectangleCircle(rectangle, shape.circles[i])) 
		{
			return true;
		}
	}
	for (int i = 0; i < shape.numRectangles; ++i) 
	{
		if (RectangleRectangle(rectangle, shape.rectangles[i]))
		{
			return true;
		}
	}
	return false;
}

bool OrientedRectangleShape(const OrientedRectangle& rect, const BoundingShape& shape)
{
	for (int i = 0; i < shape.numCircles; ++i)
	{
		if (OrientedRectangleCircle(rect, shape.circles[i])) 
		{
			return true;
		}
	}
	for (int i = 0; i < shape.numRectangles; ++i) 
	{
		if (OrientedRectangleRectangle(rect, shape.rectangles[i])) 
		{
			return true;
		}
	}
	return false;
}
