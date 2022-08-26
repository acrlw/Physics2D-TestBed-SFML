#ifndef PHYSICS2D_MINKOWSKI_H
#define PHYSICS2D_MINKOWSKI_H
#include "../../math/linear/linear.h"
namespace Physics2D
{
	struct Minkowski
	{
		Minkowski() = default;
		Minkowski(const Vec2& point_a, const Vec2& point_b) : pointA(point_a), pointB(point_b),
			result(pointA - pointB)
		{
		}

		inline bool operator ==(const Minkowski& rhs) const
		{
			return pointA == rhs.pointA && pointB == rhs.pointB;
		}

		inline bool operator !=(const Minkowski& rhs) const
		{
			return !(pointA == rhs.pointA && pointB == rhs.pointB);
		}

		Vec2 pointA;
		Vec2 pointB;
		Vec2 result;
	};
}
#endif