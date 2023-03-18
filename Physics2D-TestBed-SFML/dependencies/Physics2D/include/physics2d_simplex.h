#ifndef PHYSICS2D_SIMPLEX_H
#define PHYSICS2D_SIMPLEX_H

#include "physics2d_algorithm_2d.h"
namespace Physics2D
{
	struct SimplexVertex
	{
		SimplexVertex() = default;
		SimplexVertex(const Vector2& point_a, const Vector2& point_b) : pointA(point_a), pointB(point_b),
			result(pointA - pointB)
		{
		}

		inline bool operator ==(const SimplexVertex& rhs) const
		{
			return pointA == rhs.pointA && pointB == rhs.pointB;
		}

		inline bool operator !=(const SimplexVertex& rhs) const
		{
			return !(pointA == rhs.pointA && pointB == rhs.pointB);
		}

		Vector2 pointA;
		Vector2 pointB;
		Vector2 result;
	};
	/// <summary>
	/// Simplex structure for gjk/epa test.
	/// By convention:
	///   1 points: p0 , construct a single point
	///   2 points: p0 -> p1, construct a segment
	/// >=4 points: p0 -> p1 -> p2 -> p0, construct a polygon
	///	ATTENTION:
	///	  The performance bottleneck results in Container::Vector. Inserting and reallocating is expensive.
	/// </summary>
	/// <returns></returns>
	struct Simplex
	{
		Container::Vector<SimplexVertex> vertices;
		bool isContainOrigin = false;
		bool containOrigin(bool strict = false);
		static bool containOrigin(const Simplex& simplex, bool strict = false);

		void insert(const size_t& pos, const SimplexVertex& vertex);
		bool contains(const SimplexVertex& vertex);
		bool fuzzyContains(const SimplexVertex& vertex, const real& epsilon = 0.0001);

		Vector2 lastVertex() const;
	};
}
#endif