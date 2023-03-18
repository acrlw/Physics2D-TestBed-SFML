#ifndef PHYSICS2D_CONSTRAINT_CONTACT_H
#define PHYSICS2D_CONSTRAINT_CONTACT_H
#include <string>

#include "physics2d_body.h"
#include "physics2d_random.h"
#include "physics2d_detector.h"
namespace Physics2D
{
	struct VelocityConstraintPoint
	{
		Vec2 ra;
		Vec2 rb;
		Vec2 va;
		Vec2 vb;
		Vec2 normal;
		Vec2 tangent;
		Vec2 velocityBias;
		real bias = 0;
		real penetration = 0.0f;
		real restitution = 0.8f;
		real effectiveMassNormal = 0;
		real effectiveMassTangent = 0;
		real accumulatedNormalImpulse = 0;
		real accumulatedTangentImpulse = 0;

	};
	
	struct ContactConstraintPoint
	{
		ContactConstraintPoint() = default;
		Relation::RelationID relation = 0;
		real friction = 0.2f;
		bool active = true;
		Vec2 localA;
		Vec2 localB;
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		VelocityConstraintPoint vcp;
	};
	class ContactMaintainer
	{
	public:
		void clearAll();
		void solveVelocity(real dt);
		void solvePosition(real dt);
		void add(const Collision& collision);
		void prepare(ContactConstraintPoint& ccp, const PointPair& pair, const Collision& collision);
		void clearInactivePoints();
		void deactivateAllPoints();
		real m_maxPenetration = 0.005f;
		real m_biasFactor = 0.02f;
		bool m_blockSolver = true;
		Container::Map<Relation::RelationID, Container::Vector<ContactConstraintPoint>> m_contactTable;
	private:
	};

	
}
#endif
