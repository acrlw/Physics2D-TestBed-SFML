#ifndef PHYSICS2D_DYNAMICS_JOINT_DISTANCE_H
#define PHYSICS2D_DYNAMICS_JOINT_DISTANCE_H
#include "physics2d_joint.h"
namespace Physics2D
{
	struct PHYSICS2D_API DistanceJointPrimitive
	{
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;

		Vector2 localPointA;
		Vector2 localPointB;

		real minDistance = 0.0f;
		real maxDistance = 0.0f;

		Vector2 normal;
		real biasFactor = 0.3f;
		Vector2 bias = 0.0f;
		real effectiveMass = 0.0f;
		real accumulatedImpulse = 0.0f;

		real damping = 0.0;
		real stiffness = 0.0;
		real frequency = 5;
		real maxForce = 5000;
		real dampingRatio = 0.707f;
		real gamma = 0.0;

		real length = 0.0f;
		real currentLength = 0.0f;
	};

	class PHYSICS2D_API DistanceJoint : public Joint
	{
	public:
		DistanceJoint()
		{
			m_type = JointType::Distance;
		}
		DistanceJoint(const DistanceJointPrimitive& primitive) : m_primitive(primitive)
		{
			m_type = JointType::Distance;
		}
		void set(const DistanceJointPrimitive& primitive)
		{
			m_primitive = primitive;
		}
		void prepare(const real& dt) override
		{
			assert(m_primitive.minDistance <= m_primitive.maxDistance);

			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;

			Body* bodyA = m_primitive.bodyA;
			Body* bodyB = m_primitive.bodyB;

			real m_a = bodyA->mass();
			real m_b = bodyB->mass();
			real im_a = bodyA->inverseMass();
			real im_b = bodyB->inverseMass();
			real ii_a = bodyA->inverseInertia();
			real ii_b = bodyB->inverseInertia();

			//if (m_primitive.frequency > 0.0)
			//{
			//	//check if im_a or im_b == 0.0f
			//	real massMixing = 0.0f;
			//	if (im_a == 0.0f && im_b != 0.0f)
			//		massMixing = m_b;
			//	else if (im_a != 0.0f && im_b == 0.0f)
			//		massMixing = m_a;
			//	else
			//		massMixing = (m_a * m_b) / (m_a + m_b);

			//	real nf = naturalFrequency(m_primitive.frequency);
			//	m_primitive.stiffness = springStiffness(massMixing, nf);
			//	m_primitive.damping = springDampingCoefficient(massMixing, nf, m_primitive.dampingRatio);
			//}
			//else
			//{
			//	m_primitive.stiffness = 0.0;
			//	m_primitive.damping = 0.0;
			//}

			//m_primitive.gamma = constraintImpulseMixing(dt, m_primitive.stiffness, m_primitive.damping);
			//real erp = errorReductionParameter(dt, m_primitive.stiffness, m_primitive.damping);

			Vector2 pa = bodyA->toWorldPoint(m_primitive.localPointA);
			Vector2 ra = pa - bodyA->position();

			Vector2 pb = bodyB->toWorldPoint(m_primitive.localPointB);
			Vector2 rb = pb - bodyB->position();

			Vector2 n = (pa - pb).normal();

			real k = im_a + im_b + (ii_a * ra.cross(n) * ra.cross(n)) + (ii_b * rb.cross(n) * rb.cross(n));
			m_primitive.normal = n;
			m_primitive.effectiveMass = k > 0.0f ? 1.0f / k : 0.0f;

			m_primitive.bias = pa - pb;
			m_primitive.currentLength = (pa - pb).length();

			Vector2 P = m_primitive.accumulatedImpulse * n;
			bodyA->applyImpulse(P, ra);
			bodyB->applyImpulse(-P, rb);
		}
		void solveVelocity(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;

			Body* bodyA = m_primitive.bodyA;
			Body* bodyB = m_primitive.bodyB;

			if(m_primitive.minDistance < m_primitive.maxDistance)
			{
				
			}

			if(m_primitive.minDistance == m_primitive.maxDistance)
			{
				//equal
				Vector2 ra = bodyA->toWorldPoint(m_primitive.localPointA) - bodyA->position();
				Vector2 va = bodyA->velocity() + Vector2::crossProduct(bodyA->angularVelocity(), ra);

				Vector2 rb = bodyB->toWorldPoint(m_primitive.localPointB) - bodyB->position();
				Vector2 vb = bodyB->velocity() + Vector2::crossProduct(bodyB->angularVelocity(), rb);


				real jv = m_primitive.normal.dot(va - vb);
				real lambda = -m_primitive.effectiveMass * (jv);
				m_primitive.accumulatedImpulse += lambda;

				Vector2 P = lambda * m_primitive.normal;
				bodyA->applyImpulse(P, ra);
				bodyB->applyImpulse(-P, rb);

			}

		}
		void solvePosition(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;

			Body* bodyA = m_primitive.bodyA;
			Body* bodyB = m_primitive.bodyB;

			Vector2 pa = bodyA->toWorldPoint(m_primitive.localPointA);
			Vector2 ra = pa - bodyA->position();

			Vector2 pb = bodyB->toWorldPoint(m_primitive.localPointB);
			Vector2 rb = pb - bodyB->position();

			Vector2 error = pa - pb;
			real errorLength = error.length();
			real c = 0.0f;

			if(m_primitive.minDistance == m_primitive.maxDistance)
				c = error.length() - m_primitive.minDistance;
			
			real lambda = -m_primitive.effectiveMass * c;
			Vector2 P = lambda * error.normal();

			bodyA->position() += bodyA->inverseMass() * P;
			bodyA->rotation() += bodyA->inverseInertia() * ra.cross(P);

			bodyB->position() -= bodyB->inverseMass() * P;
			bodyB->rotation() -= bodyB->inverseInertia() * rb.cross(P);

		}

		DistanceJointPrimitive& primitive()
		{
			return m_primitive;
		}
	private:
		real m_factor = 0.4f;
		DistanceJointPrimitive m_primitive;
	};

}
#endif