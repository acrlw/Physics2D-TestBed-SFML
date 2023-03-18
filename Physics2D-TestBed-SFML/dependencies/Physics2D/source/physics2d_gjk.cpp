#include "physics2d_gjk.h"



namespace Physics2D
{


	std::tuple<bool, Simplex> GJK::gjk(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB,
	                                   const size_t& iteration)
	{

		Simplex simplex;
		bool found = false;
		Vec2 direction = transformB.position - transformA.position;
		
		if (direction.fuzzyEqual({ 0, 0 }))
			direction.set(1, 1);

		Minkowski diff = support(transformA, shapeA, transformB, shapeB, direction);
		simplex.vertices.emplace_back(diff);
		direction.negate();
		size_t iter = 0;
		Container::Vector<Minkowski> removed;
		while (iter <= iteration)
		{
			diff = support(transformA, shapeA, transformB, shapeB, direction);
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
			//if found, there is no more minkowski difference, exit loop
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

	Simplex GJK::epa(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB, const Simplex& src,
	                 const size_t& iteration, const real& epsilon)
	{

		size_t iter = 0;
		Simplex edge;
		Simplex simplex = src;
		Vec2 normal;
		Minkowski p;
		while (iter <= iteration)
		{

			auto [index1, index2] = findEdgeClosestToOrigin(simplex);

			normal = calculateDirectionByEdge(simplex.vertices[index1].result, simplex.vertices[index2].result, false).
				normal();
			
			if (GeometryAlgorithm2D::isPointOnSegment(simplex.vertices[index1].result, simplex.vertices[index2].result, { 0, 0 }))
				normal.negate();
			
			//new minkowski point
			p = support(transformA, shapeA, transformB, shapeB, normal);

			if (simplex.contains(p) || simplex.fuzzyContains(p, epsilon))
				break;

			simplex.insert(index1, p);
			iter++;
		}
		return simplex;
	}

	PenetrationInfo GJK::dumpInfo(const PenetrationSource& source)
	{
		PenetrationInfo result;
		Vec2 edge1 = source.a1 - source.b1;
		Vec2 edge2 = source.a2 - source.b2;
		Vec2 normal = calculateDirectionByEdge(edge1, edge2, false).normal();
		real originToEdge = std::fabs(normal.dot(edge1));
		result.normal = normal.negate();
		result.penetration = originToEdge;
		return result;
	}

	Minkowski GJK::support(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB, const Vec2& direction)
	{
		return Minkowski(findFarthestPoint(transformA, shapeA, direction), findFarthestPoint(transformB, shapeB, direction * -1));
	}

	std::tuple<size_t, size_t> GJK::findEdgeClosestToOrigin(const Simplex& simplex)
	{
		real min_dist = Constant::PosInfty;

		size_t index1 = 0;
		size_t index2 = 0;

		if (simplex.vertices.size() == 2)
			return std::make_tuple(0, 1);

		for (size_t i = 0; i < simplex.vertices.size() - 1; i++)
		{
			Vec2 a = simplex.vertices[i].result;
			Vec2 b = simplex.vertices[i + 1].result;

			const Vec2 p = GeometryAlgorithm2D::pointToLineSegment(a, b, {0, 0});
			const real projection = p.magnitude();


			if (min_dist > projection)
			{
				index1 = i;
				index2 = i + 1;
				min_dist = projection;
			}
			else if (realEqual(min_dist, projection))
			{
				real length1 = a.magnitudeSquare() + b.magnitudeSquare();
				real length2 = simplex.vertices[index1].result.magnitudeSquare() + simplex.vertices[index2].result.
					magnitudeSquare();
				if (length1 < length2)
				{
					index1 = i;
					index2 = i + 1;
				}
			}
		}
		return std::make_tuple(index1, index2);
	}

	Vec2 GJK::findFarthestPoint(const Transform& transform, Shape* shape, const Vec2& direction)
	{
		Vec2 target;
		Mat2 rot(-transform.rotation);
		Vec2 rot_dir = rot.multiply(direction);
		switch (shape->type())
		{
		case Shape::Type::Polygon:
		{
			const Polygon* polygon = static_cast<const Polygon*>(shape);
			auto [vertex, index] = findFarthestPoint(polygon->vertices(), rot_dir);
			target = vertex;
			break;
		}
		case Shape::Type::Circle:
		{
			const Circle* circle = static_cast<const Circle*>(shape);
			return direction.normal() * circle->radius() + transform.position;
		}
		case Shape::Type::Ellipse:
		{
			const Ellipse* ellipse = static_cast<const Ellipse*>(shape);
			target = GeometryAlgorithm2D::calculateEllipseProjectionPoint(ellipse->A(), ellipse->B(), rot_dir);
			break;
		}
		case Shape::Type::Edge:
		{
			const Edge* edge = static_cast<const Edge*>(shape);
			real dot1 = Vec2::dotProduct(edge->startPoint(), direction);
			real dot2 = Vec2::dotProduct(edge->endPoint(), direction);
			target = dot1 > dot2 ? edge->startPoint() : edge->endPoint();
			break;
		}
		case Shape::Type::Point:
		{
			return static_cast<const Point*>(shape)->position();
		}
		case Shape::Type::Capsule:
		{
			const Capsule* capsule = static_cast<const Capsule*>(shape);
			target = GeometryAlgorithm2D::calculateCapsuleProjectionPoint(capsule->width(), capsule->height(), rot_dir);
			break;
		}
		case Shape::Type::Sector:
		{
			const Sector* sector = static_cast<const Sector*>(shape);
			target = GeometryAlgorithm2D::calculateSectorProjectionPoint(sector->startRadian(), sector->spanRadian(), sector->radius(), rot_dir);
			break;
		}
		default:
			break;
		}
		rot.set(transform.rotation);
		target = rot.multiply(target);
		target += transform.position;
		return target;
	}

	std::pair<Vec2, size_t> GJK::findFarthestPoint(const Container::Vector<Vec2>& vertices, const Vec2& direction)
	{
		real max = Constant::NegInfty;
		Vec2 target;
		size_t index = 0;
		for(size_t i = 0;i < vertices.size(); i++)
		{
			real result = Vec2::dotProduct(vertices[i], direction);
			if (max < result)
			{
				max = result;
				target = vertices[i];
				index = i;
			}
		}
		return std::make_pair(target, index);
	}

	std::optional<Minkowski> GJK::adjustSimplex(Simplex& simplex, const size_t& closest_1, const size_t& closest_2)
	{
		switch (simplex.vertices.size())
		{
		case 4: //only adjust triangle from gjk
			{
				int32_t index = -1;

				for (int32_t i = 0; i < simplex.vertices.size() - 1; i++)
					if (i != closest_1 && i != closest_2)
						index = i;

				Minkowski target = simplex.vertices[index];

				simplex.vertices.erase(simplex.vertices.begin() + index);
				simplex.vertices.erase(simplex.vertices.begin() + simplex.vertices.size() - 1);
				return std::optional<Minkowski>(target);
			}
		default:
			return std::nullopt;
		}
	}

	Vec2 GJK::calculateDirectionByEdge(const Vec2& p1, const Vec2& p2, bool pointToOrigin)
	{
		const Vec2 ao = p1 * -1;
		const Vec2 ab = p2 - p1;
		Vec2 perpendicularOfAB = ab.perpendicular();
		if ((Vec2::dotProduct(ao, perpendicularOfAB) < 0 && pointToOrigin) || (
			Vec2::dotProduct(ao, perpendicularOfAB) > 0 && !pointToOrigin))
			perpendicularOfAB.negate();
		return perpendicularOfAB;
	}

	PointPair GJK::distance(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB, const real& iteration,
	                           const real& epsilon)
	{
		PointPair result;
		Simplex simplex;
		Vec2 direction = transformB.position - transformA.position;
		Minkowski m = support(transformA, shapeA, transformB, shapeB, direction);
		simplex.vertices.emplace_back(m);
		direction.negate();
		m = support(transformA, shapeA, transformB, shapeB, direction);
		simplex.vertices.emplace_back(m);
		int iter = 0;
		while (iter++ < iteration)
		{
			direction = calculateDirectionByEdge(simplex.vertices[0].result, simplex.vertices[1].result, true);
			m = support(transformA, shapeA, transformB, shapeB, direction);

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

	PenetrationSource GJK::dumpSource(const Simplex& simplex)
	{
		PenetrationSource result;
		auto [index1, index2] = findEdgeClosestToOrigin(simplex);
		result.a1 = simplex.vertices[index1].pointA;
		result.a2 = simplex.vertices[index2].pointA;
		result.b1 = simplex.vertices[index1].pointB;
		result.b2 = simplex.vertices[index2].pointB;
		return result;
	}
	
	PointPair GJK::dumpPoints(const PenetrationSource& source)
	{
		PointPair result;
		const Vec2 A_s1 = source.a1;
		const Vec2 B_s1 = source.a2;
		const Vec2 A_s2 = source.b1;
		const Vec2 B_s2 = source.b2;

		Vec2 a = source.a1 - source.b1;
		Vec2 b = source.a2 - source.b2;

		Vec2 l = b - a;
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
