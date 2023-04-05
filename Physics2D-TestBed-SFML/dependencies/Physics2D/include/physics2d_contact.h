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
		Vector2 contactLocalA;
		Vector2 contactLocalB;
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
		Vector2 positionCorrectionImpulse;
	};
	
	struct PHYSICS2D_API ContactConstraintPoint
	{
		ContactConstraintPoint() = default;
		Body::BodyPair::BodyPairID relation = 0;
		real friction = 0.2f;
		bool active = true;
		Vector2 collisionLocalA;
		Vector2 collisionLocalB;
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		VelocityConstraintPoint vcp;
	};
	class PHYSICS2D_API ContactMaintainer
	{
	public:
		void clearAll();
		void solveVelocity(real dt);
		bool solvePosition(real dt);
		void updateContact(const Collision& collision);

		void prepareContact(ContactConstraintPoint& ccp, const VertexPair& pair, const Collision& collision, const real& penetration);
		void clearInactivePoints();
		void deactivateAllPoints();
		real m_maxPenetration = 0.005f;
		real m_biasFactor = 0.2f;
		real m_contactLerpFactor = 0.70f;
		bool m_velocityBlockSolver = true;
		bool m_positionBlockSolver = false;
		Container::Map<Body::BodyPair::BodyPairID, Container::Vector<ContactConstraintPoint>> m_contactTable;


	private:
	};

	
}
#endif
