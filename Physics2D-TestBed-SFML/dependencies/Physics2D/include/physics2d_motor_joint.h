#ifndef PHYSICS2D_DYNAMICS_JOINT_MOTOR_H
#define PHYSICS2D_DYNAMICS_JOINT_MOTOR_H
#include "physics2d_joint.h"

namespace Physics2D
{
	struct PHYSICS2D_API MotorJointPrimitive
	{
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		Vector2 localPointA;
		Vector2 localPointB;

		real referenceAngle = 0.0f;
		real maxForce = 1000.0f;
		real maxTorque = 1000.0f;

		Vector2 linearError;
		real angularError = 0.0f;

		Matrix2x2 invMass1;
		real invMass2 = 0.0f;

		Vector2 accumulatedImpulse;
		real accumulatedAngularImpulse = 0.0f;

		real correctionFactor = 0.2f;
	};

	class PHYSICS2D_API MotorJoint : public Joint
	{
	public:
		MotorJoint()
		{
			m_type = JointType::Motor;
		}

		MotorJoint(const MotorJointPrimitive& prim) : m_primitive(prim)
		{
			m_type = JointType::Motor;
		}

		void set(const MotorJointPrimitive& prim)
		{
			m_primitive = prim;
		}

		void prepare(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;

			Body* bodyA = m_primitive.bodyA;
			Body* bodyB = m_primitive.bodyB;

			real im_a = bodyA->inverseMass();
			real im_b = bodyB->inverseMass();
			real ii_a = bodyA->inverseInertia();
			real ii_b = bodyB->inverseInertia();

			Vector2 pa = bodyA->toWorldPoint(m_primitive.localPointA);
			Vector2 ra = pa - bodyA->position();
			Vector2 pb = bodyB->toWorldPoint(m_primitive.localPointB);
			Vector2 rb = pb - bodyB->position();

			Matrix2x2& k = m_primitive.invMass1;
			
			k.e11() = im_a + im_b + ra.y * ra.y * ii_a + rb.y * rb.y * ii_b;
			k.e21() = -ra.y * ra.x * ii_a - rb.y * rb.x * ii_b;
			k.e12() = k.e21();
			k.e22() = im_a + im_b + ra.x * ra.x * ii_a + rb.x * rb.x * ii_b;

			k.invert();

			m_primitive.invMass2 = ii_a + ii_b > 0.0f ? 1.0f / (ii_a + ii_b) : 0.0f;

			m_primitive.linearError = pa - pb;
			m_primitive.angularError = bodyA->rotation() - bodyB->rotation() - m_primitive.referenceAngle;

			//warm start
			bodyA->angularVelocity() += m_primitive.accumulatedAngularImpulse * ii_a;
			bodyB->angularVelocity() -= m_primitive.accumulatedAngularImpulse * ii_b;

			bodyA->applyImpulse(m_primitive.accumulatedImpulse, ra);
			bodyB->applyImpulse(-m_primitive.accumulatedImpulse, rb);
		}

		void solveVelocity(const real& dt) override
		{
			if(m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;

			real inv_dt = 1.0f / dt;

			Body* bodyA = m_primitive.bodyA;
			Body* bodyB = m_primitive.bodyB;

			//solve angular first
			real dw = bodyA->angularVelocity() - bodyB->angularVelocity() + inv_dt * m_primitive.correctionFactor * m_primitive.angularError;
			real impulse = -m_primitive.invMass2 * dw;

			real oldAngImp = m_primitive.accumulatedAngularImpulse;
			real maxAngImp = m_primitive.maxTorque * dt;
			m_primitive.accumulatedAngularImpulse = oldAngImp + impulse;
			m_primitive.accumulatedAngularImpulse = Math::clamp(m_primitive.accumulatedAngularImpulse, -maxAngImp, maxAngImp);
			impulse = m_primitive.accumulatedAngularImpulse - oldAngImp;

			bodyA->angularVelocity() += bodyA->inverseInertia() * impulse;
			bodyB->angularVelocity() -= bodyB->inverseInertia() * impulse;

			Vector2 ra = bodyA->toWorldPoint(m_primitive.localPointA) - bodyA->position();
			Vector2 va = bodyA->velocity() + Vector2::crossProduct(bodyA->angularVelocity(), ra);
			Vector2 rb = bodyB->toWorldPoint(m_primitive.localPointB) - bodyB->position();
			Vector2 vb = bodyB->velocity() + Vector2::crossProduct(bodyB->angularVelocity(), rb);

			Vector2 jvb = va - vb + m_primitive.linearError * inv_dt * m_primitive.correctionFactor;
			
			Vector2 lambda = -m_primitive.invMass1.multiply(jvb);
			Vector2 oldLambda = m_primitive.accumulatedImpulse;
			m_primitive.accumulatedImpulse += lambda;
			real maxLambda = m_primitive.maxForce * dt;

			if(m_primitive.accumulatedImpulse.lengthSquare() > maxLambda * maxLambda)
			{
				m_primitive.accumulatedImpulse.normalize();
				m_primitive.accumulatedImpulse *= maxLambda;
			}

			lambda = m_primitive.accumulatedImpulse - oldLambda;

			bodyA->applyImpulse(lambda, ra);
			bodyB->applyImpulse(-lambda, rb);

		}

		void solvePosition(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;

			
		}

		MotorJointPrimitive& primitive()
		{
			return m_primitive;
		}

	private:
		MotorJointPrimitive m_primitive;
	};
}
#endif
