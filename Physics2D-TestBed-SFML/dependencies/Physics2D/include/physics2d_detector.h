#ifndef PHYSICS2D_DETECTOR_H
#define PHYSICS2D_DETECTOR_H
#include "physics2d_gjk.h"
#include "physics2d_sat.h"
#include "physics2d_mpr.h"
#include "physics2d_math.h"
#include "physics2d_shape.h"
#include "physics2d_body.h"

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
