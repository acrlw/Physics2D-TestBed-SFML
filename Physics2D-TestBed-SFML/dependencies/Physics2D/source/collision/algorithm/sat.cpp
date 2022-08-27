#include "../../../include/collision/algorithm/sat.h"

namespace Physics2D
{
	SATResult SAT::circleVsCapsule(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB)
	{
		//Convention: A is circle, B is capsule
		assert(shapeA->type() == Shape::Type::Circle);
		assert(shapeB->type() == Shape::Type::Capsule);
		SATResult result;
		return result;
	}

	SATResult SAT::circleVsSector(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB)
	{
		assert(shapeA->type() == Shape::Type::Circle);
		assert(shapeB->type() == Shape::Type::Sector);
		SATResult result;
		return result;
	}

	SATResult SAT::circleVsEdge(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB)
	{
		//Convention: A is circle, B is capsule
		assert(shapeA->type() == Shape::Type::Circle);
		assert(shapeB->type() == Shape::Type::Edge);

		SATResult result;
		Circle* circle = static_cast<Circle*>(shapeA);
		Edge* edge = static_cast<Edge*>(shapeB);

		auto* pointCircle = &result.contactPair[0].pointA;
		auto* pointEdge = &result.contactPair[0].pointB;
		
		Vec2 actualStart = transformB.position + edge->startPoint();
		Vec2 actualEnd = transformB.position + edge->endPoint();
		Vec2 normal = (actualStart - actualEnd).normal();

		if ((actualStart - transformA.position).dot(normal) < 0 &&
			(actualEnd - transformB.position).dot(normal) < 0)
			normal.negate();

		Vec2 projectedPoint = GeometryAlgorithm2D::pointToLineSegment(actualStart, actualEnd, transformA.position);
		Vec2 diff = projectedPoint - transformA.position;
		result.normal = diff.normal();
		real length = diff.magnitude();
		result.isColliding = length < circle->radius();
		result.penetration = circle->radius() - length;
		*pointCircle = transformA.position + circle->radius() * result.normal;
		*pointEdge = projectedPoint;
		result.contactPairCount++;
		return result;
	}
	

	SATResult SAT::circleVsCircle(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB)
	{
		assert(shapeA->type() == Shape::Type::Circle);
		assert(shapeB->type() == Shape::Type::Circle);

		SATResult result;
		Circle* circleA = static_cast<Circle*>(shapeA);
		Circle* circleB = static_cast<Circle*>(shapeB);
		Vec2 ba = transformA.position - transformB.position;
		real dp = circleA->radius() + circleB->radius();
		real length = ba.magnitudeSquare();
		if (length <= dp * dp)
		{
			result.normal = ba.normal();
			result.penetration = dp - length;
			result.isColliding = true;
			result.contactPair[0].pointA = transformA.position - circleA->radius() * result.normal;
			result.contactPair[0].pointB = transformB.position + circleB->radius() * result.normal;
			result.contactPairCount++;
		}
		return result;
	}

	SATResult SAT::circleVsPolygon(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB)
	{
		assert(shapeA->type() == Shape::Type::Circle);
		assert(shapeB->type() == Shape::Type::Polygon);

		Circle* circleA = static_cast<Circle*>(shapeA);
		Polygon* polygonB = static_cast<Polygon*>(shapeB);

		uint16_t collidingAxis = 0;
		SATResult result;
		
		//circle sat test
		
		//finding circle axis
		real minLength = Constant::PosInfty;
		Vec2 closest;
		for (auto& elem : polygonB->vertices())
		{
			Vec2 vertex = transformB.transform(elem);
			real length = (vertex - transformA.position).magnitudeSquare();
			if (minLength > length)
			{
				minLength = length;
				closest = vertex;
			}
		}
		Vec2 normal = closest.normal();

		ProjectedSegment segmentCircle = axisProjection(transformA, shapeA, circleA, normal);
		ProjectedSegment segmentPolygon = axisProjection(transformB, shapeB, polygonB, normal);
		
		auto [finalSegment, length] = ProjectedSegment::intersect(segmentCircle, segmentPolygon);
		
		if (length > 0)
			collidingAxis++;

		if (result.penetration > length)
		{
			result.penetration = length;
			result.normal = normal;
		}
		ProjectedPoint circlePoint, polygonPoint;
		circlePoint = segmentCircle.max == finalSegment.max ? finalSegment.max : finalSegment.min;
		polygonPoint = segmentPolygon.max == finalSegment.max ? finalSegment.max : finalSegment.min;
		
		ProjectedSegment segment;
		bool onPolygon = false;
		
		//polygon sat test
		for(int i = 0;i < polygonB->vertices().size() - 1;i++)
		{
			Vec2 v1 = transformB.transform(polygonB->vertices()[i]);
			Vec2 v2 = transformB.transform(polygonB->vertices()[i + 1]);
			Vec2 edge = v1 - v2;
			Vec2 normal = edge.perpendicular().normal();


			ProjectedSegment segmentC = axisProjection(transformA, shapeA, circleA, normal);
			ProjectedSegment segmentP = axisProjection(transformB, shapeB, polygonB, normal);

			auto [tempSegment, len] = ProjectedSegment::intersect(segmentC, segmentP);
			if (len > 0)
				collidingAxis++;

			if(result.penetration > len && len > 0)
			{
				result.penetration = len;
				result.normal = normal;
				segment = tempSegment;
				circlePoint = segmentC.max == tempSegment.max ? tempSegment.max : tempSegment.min;
			}
			
		}
		if (collidingAxis == polygonB->vertices().size())
			result.isColliding = true;
		
		result.contactPair[0].pointA = circlePoint.vertex;
		result.contactPair[0].pointB = circlePoint.vertex + -result.normal * result.penetration;
		result.contactPairCount++;


		return result;
	}

	SATResult SAT::polygonVsPolygon(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB)
	{
		assert(shapeA->type() == Shape::Type::Polygon);
		assert(shapeB->type() == Shape::Type::Polygon);

		Polygon* polyA = static_cast<Polygon*>(shapeA);
		Polygon* polyB = static_cast<Polygon*>(shapeB);
		
		SATResult result;

		//auto test = [](const Transform& transformA, Polygon* polygonA, const Transform& transformB, Polygon* polygonB)
		//{
		//	Polygon* polyA = static_cast<Polygon*>(polygonA);
		//	Polygon* polyB = static_cast<Polygon*>(polygonB);
		//	
		//	Vec2 finalNormal;
		//	real minLength = Constant::PosInfty;
		//	int collidingAxis = 0;
		//	ProjectedSegment segment;

		//	ProjectedPoint targetAPoint, targetBPoint;
		//	for(int i = 0;i < polyA->vertices().size() - 1;i++)
		//	{
		//		Vec2 v1 = transformA.transform(polyA->vertices()[i]);
		//		Vec2 v2 = transformA.transform(polyA->vertices()[i + 1]);
		//		Vec2 edge = v1 - v2;
		//		Vec2 normal = edge.perpendicular().normal();

		//		ProjectedSegment segmentA = axisProjection(transformA, polygonA, polyA, normal);
		//		ProjectedSegment segmentB = axisProjection(transformB, polygonB, polyB, normal);

		//		auto [finalSegment, length] = ProjectedSegment::intersect(segmentA, segmentB);
		//		if (length > 0)
		//			collidingAxis++;

		//		ProjectedPoint polyAPoint, polyBPoint;
		//		polyAPoint = segmentA.max == finalSegment.max ? finalSegment.max : finalSegment.min;
		//		polyBPoint = segmentB.max == finalSegment.max ? finalSegment.max : finalSegment.min;
		//		

		//		if(minLength > length)
		//		{
		//			minLength = length;
		//			finalNormal = normal;
		//			targetAPoint = polyAPoint;
		//			targetBPoint = polyBPoint;
		//		}
		//	}

		//	return std::make_tuple(finalNormal, minLength, collidingAxis, targetAPoint, targetBPoint);
		//};

		//auto [normal1, length1, axis1, polyAPoint1, polyBPoint1] = test(shapeA, shapeB);
		//auto [normal2, length2, axis2, polyBPoint2, polyAPoint2] = test(shapeB, shapeA);
		//if ( axis1 + axis2 == polyA->vertices().size() + polyB->vertices().size() - 2 )
		//	result.isColliding = true;

		//ProjectedPoint* pointA;
		//ProjectedPoint* pointB;

		//if(length1 < length2)
		//{
		//	result.penetration = length1;
		//	result.normal = normal1;
		//	pointA = &polyAPoint1;
		//	pointB = &polyBPoint1;
		//	//do clipping
		//}
		//else
		//{
		//	result.penetration = length2;
		//	result.normal = normal2;
		//	pointA = &polyAPoint2;
		//	pointB = &polyBPoint2;
		//	//do clipping
		//}
		//auto clipEdgeA = ContactGenerator::findClipEdge(polyA->vertices(), pointA->index, result.normal);
		//auto clipEdgeB = ContactGenerator::findClipEdge(polyB->vertices(), pointB->index, -result.normal);
		//auto pairList = ContactGenerator::clip(clipEdgeA, clipEdgeB, result.normal);

		return result;
	}

	SATResult SAT::polygonVsCapsule(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB)
	{
		assert(shapeA->type() == Shape::Type::Polygon);
		assert(shapeB->type() == Shape::Type::Capsule);
		return SATResult();
	}

	SATResult SAT::polygonVsSector(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB)
	{
		assert(shapeA->type() == Shape::Type::Polygon);
		assert(shapeB->type() == Shape::Type::Sector);
		return SATResult();
	}
	SATResult SAT::capsuleVsEdge(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB)
	{
		assert(shapeA->type() == Shape::Type::Capsule);
		assert(shapeB->type() == Shape::Type::Edge);
		return SATResult();
	}
	SATResult SAT::capsuleVsCapsule(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB)
	{
		assert(shapeA->type() == Shape::Type::Capsule);
		assert(shapeB->type() == Shape::Type::Capsule);
		return SATResult();
	}
	SATResult SAT::capsuleVsSector(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB)
	{
		assert(shapeA->type() == Shape::Type::Capsule);
		assert(shapeB->type() == Shape::Type::Sector);
		return SATResult();
	}
	SATResult SAT::sectorVsSector(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB)
	{
		assert(shapeA->type() == Shape::Type::Sector);
		assert(shapeB->type() == Shape::Type::Sector);
		return SATResult();
	}
	SATResult SAT::polygonVsEdge(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB)
	{
		assert(shapeA->type() == Shape::Type::Polygon);
		assert(shapeB->type() == Shape::Type::Edge);
		return SATResult();
	}
	

	ProjectedSegment SAT::axisProjection(const Transform& transform, Shape* shape, Polygon* polygon, const Vec2& normal)
	{
		ProjectedPoint minPoint, maxPoint;
		minPoint.value = Constant::PosInfty;
		maxPoint.value = Constant::NegInfty;

		for(size_t i = 0;i < polygon->vertices().size();i++)
		{
			Vec2 vertex = transform.transform(polygon->vertices()[i]);
			real value = vertex.dot(normal);

			if (value < minPoint.value)
			{
				minPoint.vertex = vertex;
				minPoint.value = value;
				minPoint.index = i;
			}

			if (value > maxPoint.value)
			{
				maxPoint.vertex = vertex;
				maxPoint.value = value;
				maxPoint.index = i;
			}
		}

		ProjectedSegment segment;
		segment.max = maxPoint;
		segment.min = minPoint;
		return segment;
	}

	ProjectedSegment SAT::axisProjection(const Transform& transform, Shape* shape, Circle* circle, const Vec2& normal)
	{
		ProjectedPoint minCircle, maxCircle;

		maxCircle.vertex = transform.position + normal * circle->radius();
		maxCircle.value = transform.position.dot(normal) + circle->radius();

		minCircle.vertex = transform.position - normal * circle->radius();
		minCircle.value = transform.position.dot(normal) - circle->radius();

		ProjectedSegment segmentCircle;

		segmentCircle.min = minCircle;
		segmentCircle.max = maxCircle;

		return segmentCircle;
	}

	ProjectedSegment SAT::axisProjection(const Transform& transform, Shape* shape, Ellipse* ellipse, const Vec2& normal)
	{
		ProjectedPoint minEllipse, maxEllipse;
		Vec2 rot_dir = Mat2(-transform.rotation).multiply(normal);
		maxEllipse.vertex = GeometryAlgorithm2D::calculateEllipseProjectionPoint(ellipse->A(), ellipse->B(), rot_dir);
		maxEllipse.vertex = transform.transform(maxEllipse.vertex);
		maxEllipse.value = maxEllipse.vertex.dot(normal);
		
		rot_dir = Mat2(-transform.rotation).multiply(-normal);
		minEllipse.vertex = GeometryAlgorithm2D::calculateEllipseProjectionPoint(ellipse->A(), ellipse->B(), rot_dir);
		minEllipse.vertex = transform.transform(minEllipse.vertex);
		minEllipse.value = minEllipse.vertex.dot(normal);
		
		ProjectedSegment segmentEllipse;

		segmentEllipse.min = minEllipse;
		segmentEllipse.max = maxEllipse;

		return segmentEllipse;
		 
	}

	ProjectedSegment SAT::axisProjection(const Transform& transform, Shape* shape, Capsule* capsule, const Vec2& normal)
	{
		ProjectedPoint min, max;
		Vec2 direction = Mat2(-transform.rotation).multiply(normal);
		Vec2 p1 = GeometryAlgorithm2D::calculateCapsuleProjectionPoint(capsule->width(), capsule->height(), direction);
		Vec2 p2 = GeometryAlgorithm2D::calculateCapsuleProjectionPoint(capsule->width(), capsule->height(), -direction);
		p1 = transform.transform(p1);
		p2 = transform.transform(p2);

		max.vertex = p1;
		max.value = max.vertex.dot(normal);
		min.vertex = p2;
		min.value = min.vertex.dot(normal);

		ProjectedSegment segment;

		segment.min = min;
		segment.max = max;

		return segment;
	}

	ProjectedSegment SAT::axisProjection(const Transform& transform, Shape* shape, Sector* sector, const Vec2& normal)
	{
		ProjectedPoint min, max;
		Vec2 direction = Mat2(-transform.rotation).multiply(normal);
		Vec2 p1 = GeometryAlgorithm2D::calculateSectorProjectionPoint(sector->startRadian(), sector->spanRadian(), sector->radius(), direction);
		Vec2 p2 = GeometryAlgorithm2D::calculateSectorProjectionPoint(sector->startRadian(), sector->spanRadian(), sector->radius(), -direction);
		p1 = transform.transform(p1);
		p2 = transform.transform(p2);

		max.vertex = p1;
		max.value = max.vertex.dot(normal);
		min.vertex = p2;
		min.value = min.vertex.dot(normal);

		ProjectedSegment segment;

		segment.min = min;
		segment.max = max;

		return segment;
	}
	
	std::tuple<ProjectedSegment, real> ProjectedSegment::intersect(const ProjectedSegment& s1, const ProjectedSegment& s2)
	{
		real difference = Constant::NegInfty;
		ProjectedSegment result;
		if(s1.min.value <= s2.min.value && s1.max.value <= s2.max.value)
		{
			difference = s1.max.value - s2.min.value;
			result.max = s1.max;
			result.min = s2.min;
		}
		else if (s2.min.value <= s1.min.value && s2.max.value <= s1.max.value)
		{
			difference = s2.max.value - s1.min.value;
			result.max = s2.max;
			result.min = s1.min;
		}
		else if(s1.min.value >= s2.min.value && s1.max.value <= s2.max.value)
		{
			if((s1.max.value - s2.min.value) > (s2.max.value - s1.min.value))
			{
				difference = s1.max.value - s2.min.value;
				result.max = s1.max;
				result.min = s2.min;
			}
			else
			{
				difference = s2.max.value - s1.min.value;
				result.max = s2.max;
				result.min = s1.min;
			}
		}
		else if (s2.min.value >= s1.min.value && s2.max.value <= s1.max.value)
		{
			if ((s2.max.value - s1.min.value) > (s1.max.value - s2.min.value))
			{
				difference = s2.max.value - s1.min.value;
				result.max = s2.max;
				result.min = s1.min;
			}
			else
			{
				difference = s1.max.value - s2.min.value;
				result.max = s1.max;
				result.min = s2.min;
			}
		}
		return std::make_tuple(result, difference);
	}
	bool ProjectedPoint::operator==(const ProjectedPoint& rhs)
	{
		return vertex.fuzzyEqual(rhs.vertex) && value == rhs.value;
	}
}
