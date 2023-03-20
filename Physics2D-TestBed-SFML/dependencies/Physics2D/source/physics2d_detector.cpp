#include "physics2d_detector.h"
namespace Physics2D
{

	bool Detector::collide(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		Simplex simplex = Narrowphase::gjk(shapeA, shapeB);
		bool isColliding = simplex.isContainOrigin;

		if (shapeA.transform.position.fuzzyEqual(shapeB.transform.position) && !isColliding)
			isColliding = simplex.containsOrigin(true);

		return isColliding;
	}
	bool Detector::collide(Body* bodyA, Body* bodyB)
	{
		assert(bodyA != nullptr && bodyB != nullptr);

		ShapePrimitive shapeA, shapeB;
		shapeA.shape = bodyA->shape();
		shapeA.transform.rotation = bodyA->rotation();
		shapeA.transform.position = bodyA->position();

		shapeB.shape = bodyB->shape();
		shapeB.transform.rotation = bodyB->rotation();
		shapeB.transform.position = bodyB->position();

		return collide(shapeA, shapeB);
	}
	bool Detector::collide(const ShapePrimitive& shapeA, Body* bodyB)
	{
		assert(shapeA.shape != nullptr && bodyB != nullptr);

		ShapePrimitive shapeB;
		shapeB.shape = bodyB->shape();
		shapeB.transform.rotation = bodyB->rotation();
		shapeB.transform.position = bodyB->position();

		return collide(shapeA, shapeB);
	}
	bool Detector::collide(Body* bodyA, const ShapePrimitive& shapeB)
	{
		assert(shapeB.shape != nullptr && bodyA != nullptr);

		ShapePrimitive shapeA;
		shapeA.shape = bodyA->shape();
		shapeA.transform.rotation = bodyA->rotation();
		shapeA.transform.position = bodyA->position();

		return collide(shapeA, shapeB);
	}
	Collision Detector::detect(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		Collision result;
		assert(shapeA.shape != nullptr && shapeB.shape != nullptr);

		Simplex simplex = Narrowphase::gjk(shapeA, shapeB);
		bool isColliding = simplex.isContainOrigin;

		if (shapeA.transform.position.fuzzyEqual(shapeB.transform.position) && !isColliding)
			isColliding = simplex.containsOrigin(true);

		if (isColliding)
		{
			CollisionInfo info = Narrowphase::epa(simplex, shapeA, shapeB);
			result.normal = info.normal;
			result.isColliding = isColliding;
			result.penetration = info.penetration;

			result.contactList = Narrowphase::generateContacts(shapeA, shapeB, info);
		}

		return result;
	}
	Collision Detector::detect(Body* bodyA, const ShapePrimitive& shapeB)
	{
		Collision result;

		assert(bodyA != nullptr && shapeB.shape != nullptr);

		ShapePrimitive shapeA;
		shapeA.shape = bodyA->shape();
		shapeA.transform.rotation = bodyA->rotation();
		shapeA.transform.position = bodyA->position();


		result = detect(shapeA, shapeB);
		result.bodyA = bodyA;
		result.bodyB = nullptr;

		return result;

	}
	Collision Detector::detect(const ShapePrimitive& shapeA, Body* bodyB)
	{
		Collision result;

		assert(shapeA.shape != nullptr && bodyB != nullptr);

		ShapePrimitive shapeB;
		shapeB.shape = bodyB->shape();
		shapeB.transform.rotation = bodyB->rotation();
		shapeB.transform.position = bodyB->position();

		result = detect(shapeA, shapeB);
		result.bodyA = nullptr;
		result.bodyB = bodyB;

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


		ShapePrimitive shapeA, shapeB;
		shapeA.shape = bodyA->shape();
		shapeA.transform.rotation = bodyA->rotation();
		shapeA.transform.position = bodyA->position();

		shapeB.shape = bodyB->shape();
		shapeB.transform.rotation = bodyB->rotation();
		shapeB.transform.position = bodyB->position();

		result = detect(shapeA, shapeB);
		result.bodyA = bodyA;
		result.bodyB = bodyB;

		return result;
	}
	PointPair Detector::distance(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{
		assert(shapeA.shape != nullptr && shapeB.shape != nullptr);
		return GJKHelper::distance(shapeA, shapeB);
	}
	PointPair Detector::distance(Body* bodyA, const ShapePrimitive& shapeB)
	{
		assert(bodyA != nullptr && shapeB.shape != nullptr);

		ShapePrimitive shapeA;
		shapeA.shape = bodyA->shape();
		shapeA.transform.rotation = bodyA->rotation();
		shapeA.transform.position = bodyA->position();

		return GJKHelper::distance(shapeA, shapeB);
	}
	PointPair Detector::distance(const ShapePrimitive& shapeA, Body* bodyB)
	{
		assert(bodyB != nullptr && shapeA.shape != nullptr);

		ShapePrimitive shapeB;
		shapeB.shape = bodyB->shape();
		shapeB.transform.rotation = bodyB->rotation();
		shapeB.transform.position = bodyB->position();

		return GJKHelper::distance(shapeA, shapeB);
	}
	PointPair Detector::distance(Body* bodyA, Body* bodyB)
	{
		PointPair result;
		assert(bodyA != nullptr && bodyB != nullptr);

		if (bodyA == bodyB)
			return result;

		ShapePrimitive shapeA, shapeB;
		shapeA.shape = bodyA->shape();
		shapeA.transform.rotation = bodyA->rotation();
		shapeA.transform.position = bodyA->position();

		shapeB.shape = bodyB->shape();
		shapeB.transform.rotation = bodyB->rotation();
		shapeB.transform.position = bodyB->position();

		return GJKHelper::distance(shapeA, shapeB);
	}
}