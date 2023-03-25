#ifndef PHYSICS2D_CONSTRAINT_CONTACT_H
#define PHYSICS2D_CONSTRAINT_CONTACT_H
#include <string>

#include "physics2d_body.h"
#include "physics2d_random.h"
#include "physics2d_detector.h"
namespace Physics2D
{
	struct PHYSICS2D_API VelocityConstraintPoint
	{
		Vector2 ra;
		Vector2 rb;
		Vector2 va;
		Vector2 vb;
		Vector2 normal;
		Vector2 tangent;
		Vector2 velocityBias;
		real bias = 0;
		real penetration = 0.0f;
		real restitution = 0.8f;
		real effectiveMassNormal = 0;
		real effectiveMassTangent = 0;
		real accumulatedNormalImpulse = 0;
		real accumulatedTangentImpulse = 0;

	};
	
	struct PHYSICS2D_API ContactConstraintPoint
	{
		ContactConstraintPoint() = default;
		Body::Relation::RelationID relation = 0;
		real friction = 0.2f;
		bool active = true;
		Vector2 localA;
		Vector2 localB;
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		VelocityConstraintPoint vcp;
	};
	class PHYSICS2D_API ContactMaintainer
	{
	public:
		void clearAll();
		void solveVelocity(real dt);
		void solvePosition(real dt);
		void add(const Collision& collision);
		void prepare(ContactConstraintPoint& ccp, const VertexPair& pair, const Collision& collision);
		void clearInactivePoints();
		void deactivateAllPoints();
		real m_maxPenetration = 0.001f;
		real m_biasFactor = 0.030f;
		bool m_blockSolver = true;
		Container::Map<Body::Relation::RelationID, Container::Vector<ContactConstraintPoint>> m_contactTable;
	private:
	};

	
}
#endif
