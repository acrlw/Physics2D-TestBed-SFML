#include "physics2d_narrowphase.h"
#include "physics2d_narrowphase.h"

namespace Physics2D
{
	Simplex Narrowphase::gjk(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const size_t& iteration)
	{
		Simplex simplex;
		Vector2 direction = shapeB.transform.position - shapeA.transform.position;

		if (direction.fuzzyEqual({ 0, 0 }))
			direction.set(1, 1);
		//first
		SimplexVertex vertex = support(shapeA, shapeB, direction);
		simplex.addSimplexVertex(vertex);
		//second
		direction.negate();
		vertex = support(shapeA, shapeB, direction);
		simplex.addSimplexVertex(vertex);
		//check 1d simplex(line segment) contains origin
		if (simplex.containsOrigin())
			return simplex;
		//third
		size_t iter = 0;
		while (iter <= iteration)
		{
			//default closest edge is index 0 and index 1
			direction = calculateDirectionByEdge(simplex.vertices[0], simplex.vertices[1], true);
			vertex = support(shapeA, shapeB, direction);

			//find repeated vertex
			if (simplex.contains(vertex))
				break;

			//vertex does not pass origin
			if (vertex.result.dot(direction) <= 0)
				break;

			simplex.addSimplexVertex(vertex);

			//use barycentric coordinates to check contains origin and find closest edge
			const SimplexVertex va = simplex.vertices[0];
			const SimplexVertex vb = simplex.vertices[1];
			const SimplexVertex vc = simplex.vertices[2];
			const Vector2 a = va.result;
			const Vector2 b = vb.result;
			const Vector2 c = vc.result;
			const Vector2 ab = b - a;
			const Vector2 ac = c - a;
			const Vector2 bc = c - b;
			const real ab_length = ab.length();
			const real ac_length = ab.length();
			const real bc_length = ab.length();

			const real u_ab = -a.dot(ab.normal()) / ab_length;
			const real u_ac = -a.dot(ac.normal()) / ac_length;
			const real u_bc = -b.dot(bc.normal()) / bc_length;
			const real v_ab = 1 - u_ab;
			const real v_ac = 1 - u_ac;
			const real v_bc = 1 - u_bc;
			/*
			 * [Ax Bx Cx][u] = [0]
			 * [Ay By Cy][v] = [0]
			 * [ 1  1  1][w] = [1]
			 * solve for u,v,w
			 */
			const real det = a.y * b.x - a.x * b.y + a.x * c.y - a.y * c.x + b.y * c.x - c.y * b.x;
			assert(det != 0.0f);
			const real u = (b.y * c.x - c.y * b.x) / det;
			const real v = (c.y * a.x - a.y * c.x) / det;
			const real w = 1 - u - v;

			
			if (u_ac > 0 && v_ac > 0 && v <= 0)
			{
				simplex.vertices[1] = vc;
			}
			else if (u_bc > 0 && v_bc > 0 && u <= 0)
			{
				simplex.vertices[0] = vc;
			}
			else if (u > 0 && v > 0 && w > 0)
			{
				//in region ABC, origin is inside simplex
				simplex.isContainOrigin = true;

				//reorder simplex index, so that the closest edge is index 0 and index 1
				//h means height
				const real h_u = u / bc_length;
				const real h_v = v / ac_length;
				const real h_w = w / ab_length;
				const real min = Math::tripleMin(h_u, h_v, h_w);
				if (min == h_u)
				{
					//c b a
					simplex.vertices[0] = vc;
					simplex.vertices[1] = va;
				}
				else if (min == h_v)
				{
					//a c b
					simplex.vertices[1] = vc;
					simplex.vertices[2] = vb;
				}

				return simplex;
			}

			simplex.removeByIndex(2);

			iter++;
		}
		return simplex;
	}

	CollisionInfo Narrowphase::epa(const Simplex& simplex, const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const size_t& iteration, const real& epsilon)
	{
		//return 1d simplex with edge closest to origin
		CollisionInfo info;
		info.simplex = simplex;
		info.simplex.removeByIndex(2);

		auto iterNext = [](std::list<SimplexVertexWithOriginDistance>::iterator& targetIter,
			std::list<SimplexVertexWithOriginDistance>& list)
		{
			++targetIter;
			if (targetIter == list.end())
				targetIter = list.begin();
		};
		auto iterPrev = [](std::list<SimplexVertexWithOriginDistance>::iterator& targetIter,
			std::list<SimplexVertexWithOriginDistance>& list)
		{
			if (targetIter == list.begin())
				targetIter = list.end();
			--targetIter;
		};


		//initiate polytope
		std::list<SimplexVertexWithOriginDistance> polytope;
		for(auto iter = simplex.vertices.begin(); iter != simplex.vertices.end(); ++iter)
		{
			auto next = iter;
			++next;
			if (next == simplex.vertices.end())
				next = simplex.vertices.begin();
			SimplexVertexWithOriginDistance pair;
			pair.vertex = *iter;
			pair.distance = GeometryAlgorithm2D::pointToLineSegment(iter->result, next->result, { 0, 0 })
				.length();
			polytope.emplace_back(pair);
		}


		size_t iter = 0;
		Vector2 direction;
		SimplexVertex vertex;

		auto iterStart = polytope.begin();
		auto iterEnd = polytope.end();
		auto iterTemp = polytope.begin();


		while(++iter < iteration)
		{
			//closest edge index is set to index 0 and index 1
			direction = calculateDirectionByEdge(info.simplex.vertices[0], info.simplex.vertices[1], false);

			vertex = support(shapeA, shapeB, direction);

			//cannot find any new vertex
			if (info.simplex.contains(vertex))
			{
				Vector2 temp = -GeometryAlgorithm2D::pointToLineSegment(info.simplex.vertices[0].result, info.simplex.vertices[1].result
					, { 0, 0 });
				info.penetration = temp.length();
				info.normal = temp.normal();
				return info;
			}

			//set to begin
			iterTemp = iterStart;

			//calculate distance
			//iterTemp->vertex = result.vertices[0];

			iterStart->distance = GeometryAlgorithm2D::pointToLineSegment(info.simplex.vertices[0].result, vertex.result, { 0,  0})
				.length();

			iterNext(iterTemp, polytope);
			//insert
			SimplexVertexWithOriginDistance pair;
			pair.vertex = vertex;
			pair.distance = GeometryAlgorithm2D::pointToLineSegment(vertex.result, iterTemp->vertex.result, { 0, 0 })
				.length();
			polytope.insert(iterTemp, pair);

			//reset iterTemp for next loop
			iterTemp = iterStart;
			//find shortest distance and set iterStart
			real minDistance = Constant::Max;
			auto iterTarget = iterStart;
			while(true)
			{
				if (iterTemp->distance < minDistance)
				{
					minDistance = iterTemp->distance;
					iterTarget = iterTemp;
				}
				iterNext(iterTemp, polytope);
				if (iterTemp == iterStart)
					break;
			}
			iterStart = iterTarget;
			iterEnd = iterTarget;
			iterPrev(iterEnd, polytope);

			//set to begin
			iterTemp = iterStart;
			iterNext(iterTemp, polytope);
			//reset simplex
			info.penetration = minDistance;
			info.simplex.vertices[0] = iterStart->vertex;
			info.simplex.vertices[1] = iterTemp->vertex;

		}
		info.normal = -GeometryAlgorithm2D::pointToLineSegment(info.simplex.vertices[0].result, info.simplex.vertices[1].result
			, { 0, 0 }).normal();
		return info;
	}

	SimplexVertex Narrowphase::support(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const Vector2& direction)
	{
		SimplexVertex vertex;
		std::tie(vertex.point[0], vertex.index[0]) = findFurthestPoint(shapeA, direction);
		std::tie(vertex.point[1], vertex.index[1]) = findFurthestPoint(shapeB, direction.negative());
		vertex.result = vertex.point[0] - vertex.point[1];
		return vertex;
	}

	std::pair<Vector2, Index> Narrowphase::findFurthestPoint(const ShapePrimitive& shape, const Vector2& direction)
	{
		Vector2 target;
		Matrix2x2 rot(-shape.transform.rotation);
		Vector2 rot_dir = rot.multiply(direction);
		Index finalIndex = INT_MAX;
		switch (shape.shape->type())
		{
		case Shape::Type::Polygon:
		{
			const Polygon* polygon = static_cast<const Polygon*>(shape.shape);
			std::tie(target, finalIndex) = findFurthestPoint(polygon->vertices(), rot_dir);
			break;
		}
		case Shape::Type::Circle:
		{
			const Circle* circle = static_cast<const Circle*>(shape.shape);
			return std::make_pair(direction.normal() * circle->radius() + shape.transform.position, finalIndex);
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
		case Shape::Type::Point:
		{
			return std::make_pair(static_cast<const Point*>(shape.shape)->position(), finalIndex);
		}
		case Shape::Type::Capsule:
		{
			const Capsule* capsule = static_cast<const Capsule*>(shape.shape);
			target = GeometryAlgorithm2D::calculateCapsuleProjectionPoint(capsule->width(), capsule->height(), rot_dir);
			break;
		}
		case Shape::Type::Sector:
		{
			const Sector* sector = static_cast<const Sector*>(shape.shape);
			target = GeometryAlgorithm2D::calculateSectorProjectionPoint(sector->startRadian(), sector->spanRadian(), sector->radius(), rot_dir);
			break;
		}
		case Shape::Type::Curve:
			assert(false && "Mapping for curve is not supported.");
			break;
		}
		rot.set(shape.transform.rotation);
		target = rot.multiply(target);
		target += shape.transform.position;
		return std::make_pair(target, finalIndex);
	}

	Vector2 Narrowphase::calculateDirectionByEdge(const SimplexVertex& v1, const SimplexVertex& v2, bool pointToOrigin)

	{
		const Vector2 p1 = v1.result;
		const Vector2 p2 = v2.result;
		const Vector2 ao = p1.negative();
		const Vector2 ab = p2 - p1;
		Vector2 perpendicularOfAB = ab.perpendicular();
		if ((Vector2::dotProduct(ao, perpendicularOfAB) < 0 && pointToOrigin) || (
			Vector2::dotProduct(ao, perpendicularOfAB) > 0 && !pointToOrigin))
			perpendicularOfAB.negate();
		return perpendicularOfAB;
	}
	std::pair<Vector2, Index> Narrowphase::findFurthestPoint(const Container::Vector<Vector2>& vertices, const Vector2& direction)
	{
		real max = Constant::NegativeMin;
		Vector2 target;
		Index index = 0;
		for (Index i = 0; i < vertices.size(); i++)
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

	ContactPair Narrowphase::clip(const Simplex& simplex, const Vector2& normal, const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		ContactPair pair;
		//find feature
		pair.feature[0] = findFeatures(simplex, normal, shapeA, 0);
		pair.feature[1] = findFeatures(simplex, normal, shapeB, 1);

		if (pair.feature[0].isValid && pair.feature[1].isValid)
		{
			//clip two edge
		}
		else if(!pair.feature[0].isValid && pair.feature[1].isValid)
		{
			//project a to edge b
			
		}
		else if(pair.feature[0].isValid && !pair.feature[1].isValid)
		{
			//project b to edge a
		}
		else
		{
			assert(false && "Invalid simplex.");
		}
		
		return pair;
	}

	real Narrowphase::gjkDistance(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const size_t& iteration)
	{
		return real();
	}

	void Narrowphase::sat(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{

	}
	void Narrowphase::satPolygonVsPolygon(const Polygon& polygonA, const Transform& transformA, const Polygon& polygonB, const Transform& transformB)
	{
	}
	void Narrowphase::satPolygonVsCircle(const Polygon& polygonA, const Transform& transformA, const Circle& circleB, const Transform& transformB)
	{
	}
	void Narrowphase::satPolygonVsEllipse(const Polygon& polygonA, const Transform& transformA, const Ellipse& ellipseB, const Transform& transformB)
	{
	}
	void Narrowphase::satPolygonVsEdge(const Polygon& polygonA, const Transform& transformA, const Edge& edgeB, const Transform& transformB)
	{
	}

	Feature Narrowphase::findFeatures(const Simplex& simplex, const Vector2& normal, const ShapePrimitive& shape, const Index& AorB)
	{
		Feature feature;
		feature.isValid = simplex.vertices[0].index[AorB] != INT_MAX;
		if (feature.isValid)
		{
			if (simplex.vertices[0].point[AorB] == simplex.vertices[1].point[AorB])
			{
				//same vertex case
				//find neighbor index

				const Polygon* polygon = static_cast<const Polygon*>(shape.shape);

				const Index tempIndex = simplex.vertices[0].index[AorB];
				const size_t realSize = polygon->vertices().size() - 1;
				//TODO: change vertex convention of polygon 
				const Index tempIndexNext = (tempIndex + 1) % realSize;
				const Index tempIndexPrev = (tempIndex - 1 + realSize) % realSize;

				//check most perpendicular
				const Vector2 ab = polygon->vertices()[tempIndexNext] - polygon->vertices()[tempIndex];
				const Vector2 ac = polygon->vertices()[tempIndex] - polygon->vertices()[tempIndexPrev];

				const Vector2 n = shape.transform.inverseRotatePoint(normal);

				const real dot1 = Math::abs(Vector2::dotProduct(ab, n));
				const real dot2 = Math::abs(Vector2::dotProduct(ac, n));

				if (dot1 > dot2)
				{
					feature.index[0] = tempIndexPrev;
					feature.index[1] = tempIndex;
				}
				else
				{
					feature.index[0] = tempIndex;
					feature.index[1] = tempIndexNext;
				}
				
			}
			else
			{
				//edge case
				feature.index[0] = simplex.vertices[0].index[AorB];
				feature.index[1] = simplex.vertices[1].index[AorB];
			}
		}
		else
		{
			//vertex case
			feature.vertex = simplex.vertices[0].point[AorB];
		}
		return feature;
	};

}
