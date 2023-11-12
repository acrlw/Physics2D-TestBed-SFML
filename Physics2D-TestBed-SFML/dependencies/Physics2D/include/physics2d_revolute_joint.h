#ifndef PHYSICS2D_DYNAMICS_JOINT_REVOLUTE_H
#define PHYSICS2D_DYNAMICS_JOINT_REVOLUTE_H
#include "physics2d_joint.h"
namespace Physics2D
{
	struct PHYSICS2D_API RevoluteJointPrimitive
	{
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		Vector2 localPointA;
		Vector2 localPointB;
		real referenceAngle = 0.0f;
		real lowerAngle = Math::degreeToRadian(-30.0f);
		real upperAngle = Math::degreeToRadian(30.0f);

		Vector2 linearError;
		real angularError = 0.0f;

		real damping = 0.0f;
		real stiffness = 0.0f;
		real frequency = 8.0f;
		real maxForce = 5000.0f;
		real dampingRatio = 0.2f;
		real gamma = 0.0f;
		Vector2 bias;
		Matrix2x2 linearMass;
		Vector2 accumulatedImpulse;
		real angularMass = 0.0f;
		real accumulatedLowerAngularImpulse = 0.0f;
		real accumulatedUpperAngularImpulse = 0.0f;

		bool angularLimit = true;
	};
	class PHYSICS2D_API RevoluteJoint : public Joint
	{
	public:
		RevoluteJoint()
		{
			m_type = JointType::Revolute;
		}
		RevoluteJoint(const RevoluteJointPrimitive& primitive)
		{
			m_type = JointType::Revolute;
			m_primitive = primitive;
		}
		void set(const RevoluteJointPrimitive& primitive)
		{
			m_primitive = primitive;
		}
		void prepare(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;
			Body* bodyA = m_primitive.bodyA;
			Body* bodyB = m_primitive.bodyB;

			real m_a = bodyA->mass();
			real im_a = bodyA->inverseMass();
			real ii_a = bodyA->inverseInertia();

			real m_b = bodyB->mass();
			real im_b = bodyB->inverseMass();
			real ii_b = bodyB->inverseInertia();

			Vector2 pa = bodyA->toWorldPoint(m_primitive.localPointA);
			Vector2 ra = pa - bodyA->position();
			Vector2 pb = bodyB->toWorldPoint(m_primitive.localPointB);
			Vector2 rb = pb - bodyB->position();

			Matrix2x2& k = m_primitive.linearMass;

			k.e11() = im_a + im_b + ra.y * ra.y * ii_a + rb.y * rb.y * ii_b;
			k.e21() = -ra.y * ra.x * ii_a - rb.y * rb.x * ii_b;
			k.e12() = k.e21();
			k.e22() = im_a + im_b + ra.x * ra.x * ii_a + rb.x * rb.x * ii_b;

			k.invert();

			m_primitive.angularMass = ii_a + ii_b > 0.0f ? 1.0f / (ii_a + ii_b) : 0.0f;

			m_primitive.linearError = pa - pb;
			m_primitive.angularError = bodyA->rotation() - bodyB->rotation() - m_primitive.referenceAngle;

			//warm start
			if(!m_primitive.angularLimit)
			{
				m_primitive.accumulatedLowerAngularImpulse = 0.0f;
				m_primitive.accumulatedUpperAngularImpulse = 0.0f;
			}
			bodyA->angularVelocity() += (m_primitive.accumulatedLowerAngularImpulse - m_primitive.accumulatedUpperAngularImpulse) * ii_a;
			bodyB->angularVelocity() -= (m_primitive.accumulatedLowerAngularImpulse - m_primitive.accumulatedUpperAngularImpulse) * ii_b;

			bodyA->applyImpulse(m_primitive.accumulatedImpulse, ra);
			bodyB->applyImpulse(-m_primitive.accumulatedImpulse, rb);

		}
		void solveVelocity(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;
			Body* bodyA = m_primitive.bodyA;
			Body* bodyB = m_primitive.bodyB;

			if(m_primitive.angularLimit)
			{

				{
					//lower
					//non-negative constraint
					real c = Math::max(0.0f, m_primitive.angularError - m_primitive.lowerAngle);
					real dw = bodyA->angularVelocity() - bodyB->angularVelocity();
					real dC = c / dt;
					dw += dC;

					real impulse = m_primitive.angularMass * -dw;
					real old = m_primitive.accumulatedLowerAngularImpulse;
					m_primitive.accumulatedLowerAngularImpulse = Math::max(impulse + old, 0);
					impulse = m_primitive.accumulatedLowerAngularImpulse - old;


					bodyA->angularVelocity() += impulse * bodyA->inverseInertia();
					bodyB->angularVelocity() -= impulse * bodyB->inverseInertia();

				}

				{
					//upper
					//non-negative constraint
					real c = Math::max(0.0f, m_primitive.upperAngle - m_primitive.angularError);
					real dw = (bodyB->angularVelocity() - bodyA->angularVelocity());
					real dC = c / dt;
					dw += dC;

					real impulse = m_primitive.angularMass * -dw;
					real old = m_primitive.accumulatedUpperAngularImpulse;
					m_primitive.accumulatedUpperAngularImpulse = Math::max(impulse + old, 0);
					impulse = m_primitive.accumulatedUpperAngularImpulse - old;


					bodyA->angularVelocity() += -impulse * bodyA->inverseInertia();
					bodyB->angularVelocity() -= -impulse * bodyB->inverseInertia();
				}
			}


			//point to point

			Vector2 ra = bodyA->toWorldPoint(m_primitive.localPointA) - bodyA->position();
			Vector2 va = bodyA->velocity() + Vector2::crossProduct(bodyA->angularVelocity(), ra);
			Vector2 rb = bodyB->toWorldPoint(m_primitive.localPointB) - bodyB->position();
			Vector2 vb = bodyB->velocity() + Vector2::crossProduct(bodyB->angularVelocity(), rb);

			Vector2 jvb = va - vb;

			Vector2 lambda = -m_primitive.linearMass.multiply(jvb);
			m_primitive.accumulatedImpulse += lambda;

			bodyA->applyImpulse(lambda, ra);
			bodyB->applyImpulse(-lambda, rb);
		}
		void solvePosition(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;
			Body* bodyA = m_primitive.bodyA;
			Body* bodyB = m_primitive.bodyB;

			real m_a = bodyA->mass();
			real im_a = bodyA->inverseMass();
			real ii_a = bodyA->inverseInertia();

			real m_b = bodyB->mass();
			real im_b = bodyB->inverseMass();
			real ii_b = bodyB->inverseInertia();

			if(m_primitive.angularLimit)
			{

				real angle = bodyA->rotation() - bodyB->rotation() - m_primitive.referenceAngle;
				real error = 0.0f;
				if (angle <= m_primitive.lowerAngle)
				{
					error = Math::clamp(angle - m_primitive.lowerAngle, Math::radianToDegree(-8), 0);
				}
				else if (angle >= m_primitive.upperAngle)
				{
					error = Math::clamp(angle - m_primitive.upperAngle, 0, Math::radianToDegree(8));
				}
				real lambda = -m_primitive.angularMass * error;
				bodyA->rotation() += lambda * ii_a;
				bodyB->rotation() -= lambda * ii_b;
			}

			// point to point

			Vector2 pa = bodyA->toWorldPoint(m_primitive.localPointA);
			Vector2 ra = pa - bodyA->position();
			Vector2 pb = bodyB->toWorldPoint(m_primitive.localPointB);
			Vector2 rb = pb - bodyB->position();

			Matrix2x2 k;

			k.e11() = im_a + im_b + ra.y * ra.y * ii_a + rb.y * rb.y * ii_b;
			k.e21() = -ra.y * ra.x * ii_a - rb.y * rb.x * ii_b;
			k.e12() = k.e21();
			k.e22() = im_a + im_b + ra.x * ra.x * ii_a + rb.x * rb.x * ii_b;

			k.invert();
			
			Vector2 impulse = -k.multiply(pa - pb);

			bodyA->position() += impulse * im_a;
			bodyA->rotation() += Vector2::crossProduct(ra, impulse) * ii_a;

			bodyB->position() -= impulse * im_b;
			bodyB->rotation() -= Vector2::crossProduct(rb, impulse) * ii_b;

		}
		RevoluteJointPrimitive& primitive()
		{
			return m_primitive;
		}
	private:
		RevoluteJointPrimitive m_primitive;
	};
}
#endif