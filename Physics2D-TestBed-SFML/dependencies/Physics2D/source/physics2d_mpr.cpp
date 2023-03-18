#include "physics2d_mpr.h"

namespace Physics2D
{
	std::tuple<Vec2, Simplex> MPR::discover(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB)
	{
		Simplex simplex;
		Vec2 centerA = Mat2(transformA.rotation).multiply(shapeA->center());
		Vec2 centerB = Mat2(transformB.rotation).multiply(shapeB->center());
		Vec2 origin = transformB.position - transformA.position;
		Minkowski v0(centerA + transformA.position, centerB + transformB.position);
		Vec2 direction = centerB - centerA + origin;
		
		if (direction.fuzzyEqual({ 0, 0 }))
			direction.set(1, 1);
		
		Minkowski v1 = GJK::support(transformA, shapeA, transformB, shapeB, direction);
		direction = GJK::calculateDirectionByEdge(v0.result, v1.result, true);
		Minkowski v2 = GJK::support(transformA, shapeA, transformB, shapeB, direction);
		simplex.vertices.emplace_back(v0);
		simplex.vertices.emplace_back(v1);
		simplex.vertices.emplace_back(v2);
		return std::make_tuple(centerB - centerA + origin, simplex);
	}

	std::tuple<bool, Simplex> MPR::refine(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB,
	                                      const Simplex& source, const Vec2& centerToOrigin, const real& iteration)
	{
		Simplex simplex = source;
		bool isColliding = false;
		Vec2 v1, v2, direction;
		real counter = 0;
		while (counter++ < iteration)
		{
			v1 = simplex.vertices[1].result;
			v2 = simplex.vertices[2].result;
			direction = GJK::calculateDirectionByEdge(v1, v2, true);
			if (direction.dot(centerToOrigin) < 0)
			{
				direction.negate();
				isColliding = true;
			}
			Minkowski newVertex = GJK::support(transformA, shapeA, transformB, shapeB, direction);

			if (v1.fuzzyEqual(newVertex.result) || v2.fuzzyEqual(newVertex.result))
				break;

			const real dist13 = GeometryAlgorithm2D::pointToLineSegment(v1, newVertex.result, { 0, 0 }).magnitudeSquare();
			const real dist23 = GeometryAlgorithm2D::pointToLineSegment(v2, newVertex.result, { 0, 0 }).magnitudeSquare();

			//bool contain1 = GeometryAlgorithm2D::triangleContainsOrigin(simplex.vertices[0].result,
			//	simplex.vertices[1].result, newVertex.result);
			//bool contain2 = GeometryAlgorithm2D::triangleContainsOrigin(simplex.vertices[0].result,
			//	simplex.vertices[2].result, newVertex.result);
			//if(contain1 == contain2)
			//{
			//}
			//else if(contain1)
			//	simplex.vertices[2] = newVertex;
			//else if (contain2)
			//	simplex.vertices[1] = newVertex;

			if (dist13 < dist23)
				simplex.vertices[2] = newVertex;
			else
				simplex.vertices[1] = newVertex;

			
			
		}
		return std::make_tuple(isColliding, simplex);
	}
}
