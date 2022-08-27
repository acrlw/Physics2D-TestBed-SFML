#ifndef PHYSICS2D_DETECTOR_H
#define PHYSICS2D_DETECTOR_H
#include "../collision/algorithm/gjk.h"
#include "../collision/algorithm/sat.h"
#include "../collision/algorithm/mpr.h"
#include "../math/math.h"
#include "../geometry/shape.h"
#include "../dynamics/body.h"

namespace Physics2D
{
	struct Collision
	{
		bool isColliding = false;
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		Container::Vector<PointPair> contactList;
		Vec2 normal;
		real penetration = 0;
	};

	class Detector
	{

	public:
		static bool collide(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB);
		static Collision detect(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB);
		static PointPair distance(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB);

		static bool collide(Body* bodyA, Body* bodyB);
		static Collision detect(Body* bodyA, Body* bodyB);
		static PointPair distance(Body* bodyA, Body* bodyB);
		
	private:
	};
	
}
#endif
