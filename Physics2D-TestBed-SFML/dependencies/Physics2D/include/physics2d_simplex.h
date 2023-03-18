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
		inline bool isEmpty()const
		{
			return pointA.isOrigin() && pointB.isOrigin() && result.isOrigin();
		}
		Vector2 pointA;
		Vector2 pointB;
		Vector2 result;
	};

	/**
	 * \brief Simplex Vertex Array for gjk/epa test
	 * bottleneck: frequently insert operation
	 */
	struct SimplexVertexArray
	{
		Container::Vector<SimplexVertex> vertices;
		bool isContainOrigin = false;
		bool containOrigin(bool strict = false);
		static bool containOrigin(const SimplexVertexArray& simplex, bool strict = false);

		void insert(const size_t& pos, const SimplexVertex& vertex);
		bool contains(const SimplexVertex& vertex);
		bool fuzzyContains(const SimplexVertex& vertex, const real& epsilon = 0.0001);

		Vector2 lastVertex() const;
	};

	/**
	 * \brief Simplex structure
	 */
	struct Simplex
	{
		SimplexVertex vertices[3];
		size_t count = 0;
		bool isContainOrigin = false;
		bool containOrigin(bool strict = false);
		static bool containOrigin(const Simplex& simplex, bool strict = false);
		bool contains(const SimplexVertex& vertex, const real& epsilon = 1e-5);
		void addSimplexVertex(const SimplexVertex& vertex);
	};
	
}
#endif