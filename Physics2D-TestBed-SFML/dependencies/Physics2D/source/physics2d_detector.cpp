#include "physics2d_detector.h"
namespace Physics2D
{

	bool Detector::collide(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB)
	{
		auto [isColliding, simplex] = GJK::gjk(transformA, shapeA, transformB, shapeB);

		if (transformA.position.fuzzyEqual(transformB.position) && !isColliding)
			isColliding = simplex.containOrigin(true);

		return isColliding;
	}
	bool Detector::collide(Body* bodyA, Body* bodyB)
	{
		assert(bodyA != nullptr && bodyB != nullptr);

		Transform transformA, transformB;
		transformA.rotation = bodyA->rotation();
		transformA.position = bodyA->position();
		
		transformB.rotation = bodyB->rotation();
		transformB.position = bodyB->position();

		return collide(transformA, bodyA->shape(), transformB, bodyB->shape());
	}
	Collision Detector::detect(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB)
	{
		Collision result;
		assert(shapeA != nullptr && shapeB != nullptr);

		auto [isColliding, simplex] = GJK::gjk(transformA, shapeA, transformB, shapeB);

		if (transformA.position.fuzzyEqual(transformB.position) && !isColliding)
			isColliding = simplex.containOrigin(true);

		result.isColliding = isColliding;


		if (isColliding)
		{
			auto oldSimplex = simplex;
			simplex = GJK::epa(transformA, shapeA, transformB, shapeB, simplex);
			PenetrationSource source = GJK::dumpSource(simplex);

			const auto info = GJK::dumpInfo(source);
			result.normal = info.normal;
			result.penetration = info.penetration;

			auto [clipEdgeA, clipEdgeB] = ContactGenerator::recognize(transformA, shapeA, transformB, shapeB, info.normal);
			auto pairList = ContactGenerator::clip(clipEdgeA, clipEdgeB, info.normal);

			bool pass = false;
			for (auto& elem : pairList)
				if (realEqual((elem.pointA - elem.pointB).magnitudeSquare(), result.penetration * result.penetration))
					pass = true;

			//if fail, there must be a deeper contact point, use it:
			if (pass)
				result.contactList = pairList;
			else
				result.contactList.emplace_back(GJK::dumpPoints(source));
		}
		assert(result.contactList.size() != 3);
		return result;
	}

	Collision Detector::detect(Body* bodyA, Body* bodyB)
	{
		Collision result;

		assert(bodyA != nullptr && bodyB != nullptr);

		if (bodyA == bodyB)
			return result;

		if (bodyA->id() > bodyB->id())
		{
			Body* temp = bodyA;
			bodyA = bodyB;
			bodyB = temp;
		}


		Transform transformA, transformB;

		result = detect(transformA, bodyA->shape(), transformB, bodyB->shape());
		result.bodyA = bodyA;
		result.bodyB = bodyB;

		return result;
	}
	PointPair Detector::distance(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB)
	{
		assert(shapeA != nullptr && shapeB != nullptr);
		return GJK::distance(transformA, shapeA, transformB, shapeB);
	}
	PointPair Detector::distance(Body* bodyA, Body* bodyB)
	{
		PointPair result;
		assert(bodyA != nullptr && bodyB != nullptr);

		if (bodyA == bodyB)
			return result;

		Transform transformA, transformB;
		transformA.rotation = bodyA->rotation();
		transformA.position = bodyA->position();

		transformB.rotation = bodyB->rotation();
		transformB.position = bodyB->position();

		return GJK::distance(transformA, bodyA->shape(), transformB, bodyB->shape());
	}
}