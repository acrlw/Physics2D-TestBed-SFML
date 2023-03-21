#ifndef PHYSICS2D_MINKOWSKI_H
#define PHYSICS2D_MINKOWSKI_H
#include "physics2d_linear.h"
namespace Physics2D
{
	struct PHYSICS2D_API MinkowskiDiff
	{
		MinkowskiDiff() = default;
		MinkowskiDiff(const Vector2& point_a, const Vector2& point_b) : pointA(point_a), pointB(point_b),
			result(pointA - pointB)
		{
		}

		inline bool operator ==(const MinkowskiDiff& rhs) const
		{
			return pointA == rhs.pointA && pointB == rhs.pointB;
		}

		inline bool operator !=(const MinkowskiDiff& rhs) const
		{
			return !(pointA == rhs.pointA && pointB == rhs.pointB);
		}

		Vector2 pointA;
		Vector2 pointB;
		Vector2 result;
	};

}
#endif