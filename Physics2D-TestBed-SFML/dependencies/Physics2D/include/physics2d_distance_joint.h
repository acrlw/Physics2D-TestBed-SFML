#ifndef PHYSICS2D_DYNAMICS_JOINT_DISTANCE_H
#define PHYSICS2D_DYNAMICS_JOINT_DISTANCE_H
#include "physics2d_joint.h"
namespace Physics2D
{
	struct DistanceJointPrimitive
	{
		Body* bodyA = nullptr;
		Vec2 localPointA;
		Vec2 targetPoint;
		Vec2 normal;
		real biasFactor = 0.3f;
		real bias = 0.0f;
		real minDistance = 0.0f;
		real maxDistance = 0.0f;
		real effectiveMass = 0.0f;
		real accumulatedImpulse = 0.0f;
	};
	struct DistanceConstraintPrimitive
	{
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		Vec2 nearestPointA;
		Vec2 nearestPointB;
		Vec2 ra;
		Vec2 rb;
		Vec2 bias;
		Mat2 effectiveMass;
		Vec2 impulse;
		real maxForce = 200.0f;
	};
	class DistanceJoint : public Joint
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
			Body* bodyA = m_primitive.bodyA;
			Vec2 pa = bodyA->toWorldPoint(m_primitive.localPointA);
			Vec2 ra = pa - bodyA->position();
			Vec2 pb = m_primitive.targetPoint;
			real im_a = m_primitive.bodyA->inverseMass();
			real ii_a = m_primitive.bodyA->inverseInertia();
			Vec2 error = pb - pa;
			real length = error.magnitude();
			real c = 0;

			m_primitive.normal = error.normal();
			if (length < m_primitive.minDistance)
			{
				c = m_primitive.minDistance - length;
				m_primitive.normal.negate();
			}
			else if (length > m_primitive.maxDistance)
			{
				c = length - m_primitive.maxDistance;
			}
			else
			{
				m_primitive.accumulatedImpulse = 0;
				m_primitive.normal.clear();
				m_primitive.bias = 0;
				return;
			}
			if (m_primitive.bodyA->velocity().dot(m_primitive.normal) > 0)
			{
				m_primitive.accumulatedImpulse = 0;
				m_primitive.normal.clear();
				m_primitive.bias = 0;
				return;
			}
			real rn_a = m_primitive.normal.dot(ra);
			m_primitive.effectiveMass = 1.0f / (im_a + ii_a * rn_a * rn_a);
 			m_primitive.bias = m_primitive.biasFactor * c / dt;

			//Vec2 impulse = m_primitive.accumulatedImpulse * m_primitive.normal;
			//m_primitive.bodyA->applyImpulse(impulse, ra);
			
		}
		void solveVelocity(const real& dt) override
		{
			if (m_primitive.bias == 0)
				return ;
			Vec2 ra = m_primitive.bodyA->toWorldPoint(m_primitive.localPointA) - m_primitive.bodyA->position();
			Vec2 va = m_primitive.bodyA->velocity() + Vec2::crossProduct(m_primitive.bodyA->angularVelocity(), ra);

			Vec2 dv = va;
			real jv = m_primitive.normal.dot(dv);
			real jvb = -jv + m_primitive.bias;
			real lambda_n = m_primitive.effectiveMass * jvb;

			real oldImpulse = m_primitive.accumulatedImpulse;
			m_primitive.accumulatedImpulse = max(oldImpulse + lambda_n, 0);
			lambda_n = m_primitive.accumulatedImpulse - oldImpulse;

			Vec2 impulse = lambda_n * m_primitive.normal;
			m_primitive.bodyA->applyImpulse(impulse, ra);
			//m_primitive.bodyA->velocity() += m_primitive.bodyA->inverseMass() * impulse;
			//m_primitive.bodyA->angularVelocity() += m_primitive.bodyA->inverseInertia() * ra.cross(impulse);
		}
		void solvePosition(const real& dt) override
		{
			
		}

		DistanceJointPrimitive& primitive()
		{
			return m_primitive;
		}
	private:
		real m_factor = 0.4f;
		DistanceJointPrimitive m_primitive;
	};
	class DistanceConstraint : public Joint
	{
		DistanceConstraint()
		{
		}
		DistanceConstraint(const DistanceConstraintPrimitive& primitive) : m_primitive(primitive)
		{
		}
		void set(const DistanceConstraintPrimitive& primitive)
		{
			m_primitive = primitive;
		}
		void prepare(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;

			Body* bodyA = m_primitive.bodyB;
			Body* bodyB = m_primitive.bodyB;

			real im_a = bodyA->inverseMass();
			real ii_a = bodyA->inverseInertia();

			real im_b = bodyB->inverseMass();
			real ii_b = bodyB->inverseInertia();

			m_primitive.ra = m_primitive.nearestPointA - bodyA->position();
			m_primitive.rb = m_primitive.nearestPointB - bodyB->position();
			Vec2& ra = m_primitive.ra;
			Vec2& rb = m_primitive.rb;
			Vec2 error = m_primitive.nearestPointA - m_primitive.nearestPointB;

			Mat2 k;
			k[0][0] = im_a + ra.y * ra.y * ii_a + im_b + rb.y * rb.y * ii_b;
			k[1][0] = -ra.x * ra.y * ii_a - rb.x * rb.y * ii_b;
			k[0][1] = k[1][0];
			k[1][1] = im_a + ra.x * ra.x * ii_a + im_b + rb.x * rb.x * ii_b;
			
			m_primitive.bias = error * m_factor;
			m_primitive.effectiveMass = k.invert();

		}
		void solveVelocity(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;
			Vec2 va = m_primitive.bodyA->velocity() + Vec2::crossProduct(m_primitive.bodyA->angularVelocity(), m_primitive.ra);
			Vec2 vb = m_primitive.bodyB->velocity() + Vec2::crossProduct(m_primitive.bodyB->angularVelocity(), m_primitive.rb);

			Vec2 jvb = va - vb;
			jvb += m_primitive.bias;
			jvb.negate();
			Vec2 J = m_primitive.effectiveMass.multiply(jvb);
			Vec2 oldImpulse = m_primitive.impulse;
			m_primitive.impulse += J;
			real maxImpulse = dt * m_primitive.maxForce;
			if (m_primitive.impulse.magnitudeSquare() > maxImpulse * maxImpulse)
			{
				m_primitive.impulse.normalize();
				m_primitive.impulse *= maxImpulse;
			}
			J = m_primitive.impulse - oldImpulse;
			m_primitive.bodyA->applyImpulse(J, m_primitive.ra);
			m_primitive.bodyB->applyImpulse(-J, m_primitive.rb);

		}
		void set(const Vec2& pointA, const Vec2& pointB)
		{
			m_primitive.nearestPointA = pointA;
			m_primitive.nearestPointB = pointB;
		}
		void solvePosition(const real& dt) override
		{


		}

		DistanceConstraintPrimitive primitive()const
		{
			return m_primitive;
		}
	private:
		DistanceConstraintPrimitive m_primitive;
		real m_factor = 0.1f;
	};
}
#endif