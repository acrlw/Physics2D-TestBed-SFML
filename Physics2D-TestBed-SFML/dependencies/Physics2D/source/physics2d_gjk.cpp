#include "physics2d_gjk.h"



namespace Physics2D
{


	std::tuple<bool, SimplexVertexArray> GJKHelper::gjk(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
	                                   const size_t& iteration)
	{
		SimplexVertexArray simplex;
		bool found = false;
		Vector2 direction = shapeB.transform.position - shapeA.transform.position;
		
		if (direction.fuzzyEqual({ 0, 0 }))
			direction.set(1, 1);

		SimplexVertex diff = support(shapeA, shapeB, direction);
		simplex.vertices.emplace_back(diff);
		direction.negate();
		size_t iter = 0;
		Container::Vector<SimplexVertex> removed;
		while (iter <= iteration)
		{
			diff = support(shapeA, shapeB, direction);
			simplex.vertices.emplace_back(diff);
			if (simplex.vertices.size() == 3)
				simplex.vertices.emplace_back(simplex.vertices[0]);

			if (simplex.lastVertex().dot(direction) <= 0)
				break;
			if (simplex.containOrigin(true))
			{
				found = true;
				break;
			}
			//if not contain origin
			//find edge closest to origin
			//reconstruct simplex
			//find the point that is not belong to the edge closest to origin
			//if found, there is no more SimplexVertex difference, exit loop
			//if not, add the point to the list

			auto [index1, index2] = findEdgeClosestToOrigin(simplex);
			direction = calculateDirectionByEdge(simplex.vertices[index1].result,
			                                     simplex.vertices[index2].result, true);

			auto result = adjustSimplex(simplex, index1, index2);
			if (result.has_value())
			{
				if (std::find(std::begin(removed), std::end(removed), result.value()) != removed.end())
					break;

				removed.emplace_back(result.value());
			}
			iter++;
		}

		return std::make_tuple(found, simplex);
	}

	SimplexVertexArray GJKHelper::epa(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const SimplexVertexArray& src,
	                 const size_t& iteration, const real& epsilon)
	{
		size_t iter = 0;
		SimplexVertexArray edge;
		SimplexVertexArray simplex = src;
		Vector2 normal;
		SimplexVertex p;
		while (iter <= iteration)
		{

			auto [index1, index2] = findEdgeClosestToOrigin(simplex);

			normal = calculateDirectionByEdge(simplex.vertices[index1].result, simplex.vertices[index2].result, false).
				normal();
			
			if (GeometryAlgorithm2D::isPointOnSegment(simplex.vertices[index1].result, simplex.vertices[index2].result, { 0, 0 }))
				normal.negate();
			
			//new SimplexVertex point
			p = support(shapeA, shapeB, normal);

			if (simplex.contains(p) || simplex.fuzzyContains(p, epsilon))
				break;

			simplex.insert(index1, p);
			iter++;
		}
		return simplex;
	}

	PenetrationInfo GJKHelper::dumpInfo(const PenetrationSource& source)
	{
		PenetrationInfo result;
		Vector2 edge1 = source.a1 - source.b1;
		Vector2 edge2 = source.a2 - source.b2;
		Vector2 normal = calculateDirectionByEdge(edge1, edge2, false).normal();
		real originToEdge = std::fabs(normal.dot(edge1));
		result.normal = normal.negate();
		result.penetration = originToEdge;
		return result;
	}

	SimplexVertex GJKHelper::support(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const Vector2& direction)
	{
		return SimplexVertex(findFurthestPoint(shapeA, direction), findFurthestPoint(shapeB, direction * -1));
	}

	std::tuple<size_t, size_t> GJKHelper::findEdgeClosestToOrigin(const SimplexVertexArray& simplex)
	{
		real min_dist = Constant::Max;

		size_t index1 = 0;
		size_t index2 = 0;

		if (simplex.vertices.size() == 2)
			return std::make_tuple(0, 1);

		for (size_t i = 0; i < simplex.vertices.size() - 1; i++)
		{
			Vector2 a = simplex.vertices[i].result;
			Vector2 b = simplex.vertices[i + 1].result;

			const Vector2 p = GeometryAlgorithm2D::pointToLineSegment(a, b, {0, 0});
			const real projection = p.length();


			if (min_dist > projection)
			{
				index1 = i;
				index2 = i + 1;
				min_dist = projection;
			}
			else if (realEqual(min_dist, projection))
			{
				real length1 = a.lengthSquare() + b.lengthSquare();
				real length2 = simplex.vertices[index1].result.lengthSquare() + simplex.vertices[index2].result.
					lengthSquare();
				if (length1 < length2)
				{
					index1 = i;
					index2 = i + 1;
				}
			}
		}
		return std::make_tuple(index1, index2);
	}

	Vector2 GJKHelper::findFurthestPoint(const ShapePrimitive& shape, const Vector2& direction)
	{
		Vector2 target;
		Matrix2x2 rot(-shape.transform.rotation);
		Vector2 rot_dir = rot.multiply(direction);
		switch (shape.shape->type())
		{
		case Shape::Type::Polygon:
		{
			const Polygon* polygon = static_cast<const Polygon*>(shape.shape);
			auto [vertex, index] = findFurthestPoint(polygon->vertices(), rot_dir);
			target = vertex;
			break;
		}
		case Shape::Type::Circle:
		{
			const Circle* circle = static_cast<const Circle*>(shape.shape);
			return direction.normal() * circle->radius() + shape.transform.position;
		}
		case Shape::Type::Ellipse:
		{
			const Ellipse* ellipse = static_cast<const Ellipse*>(shape.shape);
			target = GeometryAlgorithm2D::calculateEllipseProjectionPoint(ellipse->A(), ellipse->B(), rot_dir);
			break;
		}
		case Shape::Type::Edge:
		{
			const Edge* edge = static_cast<const Edge*>(shape.shape);
			real dot1 = Vector2::dotProduct(edge->startPoint(), direction);
			real dot2 = Vector2::dotProduct(edge->endPoint(), direction);
			target = dot1 > dot2 ? edge->startPoint() : edge->endPoint();
			break;
		}
		case Shape::Type::Capsule:
		{
			const Capsule* capsule = static_cast<const Capsule*>(shape.shape);
			target = GeometryAlgorithm2D::calculateCapsuleProjectionPoint(capsule->width(), capsule->height(), rot_dir);
			break;
		}
		default:
			break;
		}
		rot.set(shape.transform.rotation);
		target = rot.multiply(target);
		target += shape.transform.position;
		return target;
	}

	std::pair<Vector2, size_t> GJKHelper::findFurthestPoint(const Container::Vector<Vector2>& vertices, const Vector2& direction)
	{
		real max = Constant::NegativeMin;
		Vector2 target;
		size_t index = 0;
		for(size_t i = 0;i < vertices.size(); i++)
		{
			real result = Vector2::dotProduct(vertices[i], direction);
			if (max < result)
			{
				max = result;
				target = vertices[i];
				index = i;
			}
		}
		return std::make_pair(target, index);
	}

	std::optional<SimplexVertex> GJKHelper::adjustSimplex(SimplexVertexArray& simplex, const size_t& closest_1, const size_t& closest_2)
	{
		switch (simplex.vertices.size())
		{
		case 4: //only adjust triangle from gjk
			{
				int32_t index = -1;

				for (int32_t i = 0; i < simplex.vertices.size() - 1; i++)
					if (i != closest_1 && i != closest_2)
						index = i;

				SimplexVertex target = simplex.vertices[index];

				simplex.vertices.erase(simplex.vertices.begin() + index);
				simplex.vertices.erase(simplex.vertices.begin() + simplex.vertices.size() - 1);
				return std::optional<SimplexVertex>(target);
			}
		default:
			return std::nullopt;
		}
	}

	Vector2 GJKHelper::calculateDirectionByEdge(const Vector2& p1, const Vector2& p2, bool pointToOrigin)
	{
		const Vector2 ao = p1 * -1;
		const Vector2 ab = p2 - p1;
		Vector2 perpendicularOfAB = ab.perpendicular();
		if ((Vector2::dotProduct(ao, perpendicularOfAB) < 0 && pointToOrigin) || (
			Vector2::dotProduct(ao, perpendicularOfAB) > 0 && !pointToOrigin))
			perpendicularOfAB.negate();
		return perpendicularOfAB;
	}

	PointPair GJKHelper::distance(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const real& iteration,
	                           const real& epsilon)
	{
		PointPair result;
		SimplexVertexArray simplex;
		Vector2 direction = shapeB.transform.position - shapeA.transform.position;
		SimplexVertex m = support(shapeA, shapeB, direction);
		simplex.vertices.emplace_back(m);
		direction.negate();
		m = support(shapeA, shapeB, direction);
		simplex.vertices.emplace_back(m);
		int iter = 0;
		while (iter++ < iteration)
		{
			direction = calculateDirectionByEdge(simplex.vertices[0].result, simplex.vertices[1].result, true);
			m = support(shapeA, shapeB, direction);

			if (simplex.contains(m))
				break;

			//for ellipse
			if (simplex.fuzzyContains(m, epsilon))
				break;

			simplex.vertices.emplace_back(m);
			simplex.vertices.emplace_back(simplex.vertices[0]);
			auto [index1, index2] = findEdgeClosestToOrigin(simplex);
			adjustSimplex(simplex, index1, index2);
		}
		
		return dumpPoints(dumpSource(simplex));
	}

	PenetrationSource GJKHelper::dumpSource(const SimplexVertexArray& simplex)
	{
		PenetrationSource result;
		auto [index1, index2] = findEdgeClosestToOrigin(simplex);
		result.a1 = simplex.vertices[index1].point[0];
		result.a2 = simplex.vertices[index2].point[0];
		result.b1 = simplex.vertices[index1].point[1];
		result.b2 = simplex.vertices[index2].point[1];
		return result;
	}
	
	PointPair GJKHelper::dumpPoints(const PenetrationSource& source)
	{
		PointPair result;
		const Vector2 A_s1 = source.a1;
		const Vector2 B_s1 = source.a2;
		const Vector2 A_s2 = source.b1;
		const Vector2 B_s2 = source.b2;

		Vector2 a = source.a1 - source.b1;
		Vector2 b = source.a2 - source.b2;

		Vector2 l = b - a;
		real ll = l.dot(l);
		real la = l.dot(a);
		real lambda2 = -la / ll;
		real lambda1 = 1 - lambda2;

		result.pointA.set(lambda1 * A_s1 + lambda2 * B_s1);
		result.pointB.set(lambda1 * A_s2 + lambda2 * B_s2);

		if(l.fuzzyEqual({0, 0}) || lambda2 < 0)
		{
			result.pointA.set(A_s1);
			result.pointB.set(A_s2);
		}
		if(lambda1 < 0)
		{
			result.pointA.set(B_s1);
			result.pointB.set(B_s2);
		}
		return result;
	}


}


