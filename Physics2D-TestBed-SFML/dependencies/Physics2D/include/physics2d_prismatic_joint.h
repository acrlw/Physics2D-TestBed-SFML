#ifndef PHYSICS2D_DYNAMICS_JOINT_PRISMATIC_H
#define PHYSICS2D_DYNAMICS_JOINT_PRISMATIC_H
#include "physics2d_joint.h"
namespace Physics2D
{
	struct PHYSICS2D_API PrismaticJointPrimitive
	{
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		Vector2 localPointA;
		Vector2 localPointB;
		Vector2 xAxis{1.0f, 0.0f};
		Vector2 yAxis{0.0f, 1.0f};
		real referenceAngle = 0.0f;

		real translation = 0.0f;

		real lowerLimit = -2.0f;
		real upperLimit = 2.0f;

		Matrix2x2 invK;
		real effectiveMass = 0.0f;

		real lowerImpulse = 0.0f;
		real upperImpulse = 0.0f;

		Vector2 impulse;

		Matrix3x3 invJMJt;
	};
	class PHYSICS2D_API PrismaticJoint : public Joint
	{
	public:
		PrismaticJoint()
		{
			m_type = JointType::Prismatic;
		}
		PrismaticJoint(const PrismaticJointPrimitive& primitive)
		{
			m_type = JointType::Prismatic;
			m_primitive = primitive;
		}
		void set(const PrismaticJointPrimitive& primitive)
		{
			m_primitive = primitive;
		}
		void prepare(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;
			Body* bodyA = m_primitive.bodyA;
			Body* bodyB = m_primitive.bodyB;
			m_primitive.yAxis.set(-m_primitive.xAxis.y, m_primitive.xAxis.x);

			real im_a = bodyA->inverseMass();
			real ii_a = bodyA->inverseInertia();
			real im_b = bodyB->inverseMass();
			real ii_b = bodyB->inverseInertia();

			Vector2 pa = bodyA->toWorldPoint(m_primitive.localPointA);
			Vector2 pb = bodyB->toWorldPoint(m_primitive.localPointB);
			Vector2 ra = pa - bodyA->position();
			Vector2 rb = pb - bodyB->position();

			Vector2 n = m_primitive.yAxis;
			Vector2 t = m_primitive.xAxis;
			Vector2 d = pa - pb;
			
			m_primitive.translation = t.dot(d);

			Matrix2x2& k = m_primitive.invK;
			k.e11() = im_a + im_b + ii_a * ra.cross(n) * ra.cross(n) + ii_b * rb.cross(n) * rb.cross(n);
			k.e12() = ii_a * ra.cross(n) + ii_b * rb.cross(n);
			k.e21() = k.e12();
			k.e22() = ii_a + ii_b;
			k.invert();

			//Matrix3x3& jmjt = m_primitive.invJMJt;

			//jmjt.e11() = im_a + im_b + ii_a * ra.cross(n) * ra.cross(n) + ii_b * rb.cross(n) * rb.cross(n);
			//jmjt.e12() = ii_a * ra.cross(n) * ra.cross(t) + ii_b * rb.cross(n) * rb.cross(t);
			//jmjt.e13() = ii_a * ra.cross(n) + ii_b * rb.cross(n);
			//jmjt.e21() = jmjt.e12();
			//jmjt.e22() = ii_a + ii_b + ii_a * ra.cross(t) * ra.cross(t) + ii_b * rb.cross(t) * rb.cross(t);
			//jmjt.e23() = ii_a * ra.cross(t) + ii_b * rb.cross(t);
			//jmjt.e31() = jmjt.e13();
			//jmjt.e32() = jmjt.e23();
			//jmjt.e33() = ii_a + ii_b;

			//jmjt.invert();

			m_primitive.effectiveMass = 1.0f / (im_a + im_b + ii_a * ra.cross(t) * ra.cross(t) + ii_b * rb.cross(t) * rb.cross(t));


			bodyA->angularVelocity() += m_primitive.impulse.y * ii_a;
			bodyB->angularVelocity() -= m_primitive.impulse.y * ii_b;

			Vector2 P1 = n * m_primitive.impulse.x;
			Vector2 P2 = t * (m_primitive.lowerImpulse + (-m_primitive.upperImpulse));

			bodyA->applyImpulse(P1 + P2, ra);
			bodyB->applyImpulse(-P1 - P2, rb);

		}
		void solveVelocity(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;
			Body* bodyA = m_primitive.bodyA;
			Body* bodyB = m_primitive.bodyB;

			Vector2 ra = bodyA->toWorldPoint(m_primitive.localPointA) - bodyA->position();
			Vector2 rb = bodyB->toWorldPoint(m_primitive.localPointB) - bodyB->position();

			Vector2 n = m_primitive.yAxis;
			Vector2 t = m_primitive.xAxis;

			{
				//lower limit
				real C = Math::max(m_primitive.translation - m_primitive.lowerLimit, 0.0f);
				Vector2 va = bodyA->velocity() + Vector2::crossProduct(bodyA->angularVelocity(), ra);
				Vector2 vb = bodyB->velocity() + Vector2::crossProduct(bodyB->angularVelocity(), rb);
				real tv = t.dot(va - vb);

				real dC = C / dt;
				real lambda = -m_primitive.effectiveMass * (tv + dC);
				real old = m_primitive.lowerImpulse;
				m_primitive.lowerImpulse = Math::max(0.0f, old + lambda);
				lambda = m_primitive.lowerImpulse - old;

				if (C > 0.0f && lambda != 0.0f)
					int a = 0;

				Vector2 P = lambda * t;
				bodyA->applyImpulse(P, ra);
				bodyB->applyImpulse(-P, rb);
			}

			{
				//upper limit
				real C = Math::max(m_primitive.upperLimit - m_primitive.translation, 0.0f);

				Vector2 va = bodyA->velocity() + Vector2::crossProduct(bodyA->angularVelocity(), ra);
				Vector2 vb = bodyB->velocity() + Vector2::crossProduct(bodyB->angularVelocity(), rb);
				real tv = t.dot(vb - va);
				real dC = C / dt;
				
				real lambda = -m_primitive.effectiveMass * (tv + dC);
				real old = m_primitive.upperImpulse;
				m_primitive.upperImpulse = Math::max(0.0f, old + lambda);
				lambda = m_primitive.upperImpulse - old;

				if (C > 0.0f && lambda != 0.0f)
					int a = 0;

				Vector2 P = -lambda * t;
				bodyA->applyImpulse(P, ra);
				bodyB->applyImpulse(-P, rb);
			}

			{
				//prismatic constraint

				Vector2 va = bodyA->velocity() + Vector2::crossProduct(bodyA->angularVelocity(), ra);
				Vector2 vb = bodyB->velocity() + Vector2::crossProduct(bodyB->angularVelocity(), rb);

				real nv = n.dot(va - vb);
				real dw = bodyA->angularVelocity() - bodyB->angularVelocity();

				Vector2 dv(nv, dw);
				Vector2 lambda = m_primitive.invK.multiply(-dv);
				m_primitive.impulse += lambda;
				Vector2 P1 = n * lambda.x;

				bodyA->angularVelocity() += bodyA->inverseInertia() * lambda.y;
				bodyB->angularVelocity() -= bodyB->inverseInertia() * lambda.y;

				bodyA->applyImpulse(P1, ra);
				bodyB->applyImpulse(-P1, rb);
			}

			//Vector2 va = bodyA->velocity() + Vector2::crossProduct(bodyA->angularVelocity(), ra);
			//Vector2 vb = bodyB->velocity() + Vector2::crossProduct(bodyB->angularVelocity(), rb);
			//real nv = n.dot(va - vb);
			//real tv = t.dot(va - vb);
			//real dw = bodyA->angularVelocity() - bodyB->angularVelocity();

			//Vector3 jv(nv, tv, dw);
			//Vector3 lambda = m_primitive.invJMJt.multiply(-jv);

			//bodyA->angularVelocity() += bodyA->inverseInertia() * lambda.z;
			//bodyB->angularVelocity() -= bodyB->inverseInertia() * lambda.z;

			//Vector2 P1 = n * lambda.x;
			//
			//lambda.y = 0.0f;

			//Vector2 P2 = t * lambda.y;


			//bodyA->applyImpulse(P1 + P2, ra);
			//bodyB->applyImpulse(-P1 - P2, rb);

			//m_primitive.impulse.x += lambda.x;
			//m_primitive.impulse.y += lambda.z;

		}
		void solvePosition(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;
			Body* bodyA = m_primitive.bodyA;
			Body* bodyB = m_primitive.bodyB;

			real im_a = bodyA->inverseMass();
			real ii_a = bodyA->inverseInertia();
			real im_b = bodyB->inverseMass();
			real ii_b = bodyB->inverseInertia();

			Vector2 pa = bodyA->toWorldPoint(m_primitive.localPointA);
			Vector2 pb = bodyB->toWorldPoint(m_primitive.localPointB);
			Vector2 ra = pa - bodyA->position();
			Vector2 rb = pb - bodyB->position();

			Vector2 d = pa - pb;

			Vector2 n = m_primitive.yAxis;
			Vector2 t = m_primitive.xAxis;

			real linearError = n.dot(d);
			real angularError = bodyA->rotation() - bodyB->rotation() - m_primitive.referenceAngle;
			real translationError = t.dot(d);
			real c = 0.0f;

			if(translationError < m_primitive.lowerLimit)
			{
				c = translationError - m_primitive.lowerLimit;
			}
			else if(translationError > m_primitive.upperLimit)
			{
				c = translationError - m_primitive.upperLimit;
			}

			if(c != 0.0f)
			{
				Vector3 dv(linearError, c, angularError);

				Matrix3x3 k;

				k.e11() = im_a + im_b + ii_a * ra.cross(n) * ra.cross(n) + ii_b * rb.cross(n) * rb.cross(n);
				k.e12() = ii_a * ra.cross(n) * ra.cross(t) + ii_b * rb.cross(n) * rb.cross(t);
				k.e13() = ii_a * ra.cross(n) + ii_b * rb.cross(n);
				k.e21() = k.e12();
				k.e22() = ii_a + ii_b + ii_a * ra.cross(t) * ra.cross(t) + ii_b * rb.cross(t) * rb.cross(t);
				k.e23() = ii_a * ra.cross(t) + ii_b * rb.cross(t);
				k.e31() = k.e13();
				k.e32() = k.e23();
				k.e33() = ii_a + ii_b;

				k.invert();

				Vector3 lambda = k.multiply(-dv);

				Vector2 P1 = n * lambda.x;
				Vector2 P2 = t * lambda.y;

				bodyA->rotation() += bodyA->inverseInertia() * lambda.z;
				bodyB->rotation() -= bodyB->inverseInertia() * lambda.z;

				bodyA->position() += (P1 + P2) * im_a;
				bodyB->position() -= (P1 + P2) * im_b;
			}
			else
			{
				//no translation error
				Matrix2x2 k;
				k.e11() = im_a + im_b + ii_a * ra.cross(n) * ra.cross(n) + ii_b * rb.cross(n) * rb.cross(n);
				k.e12() = ii_a * ra.cross(n) + ii_b * rb.cross(n);
				k.e21() = k.e12();
				k.e22() = ii_a + ii_b;
				k.invert();

				Vector2 dv(linearError, angularError);

				Vector2 lambda = k.multiply(-dv);

				Vector2 P1 = n * lambda.x;

				bodyA->rotation() += bodyA->inverseInertia() * lambda.y;
				bodyB->rotation() -= bodyB->inverseInertia() * lambda.y;

				bodyA->position() += P1 * im_a;
				bodyB->position() -= P1 * im_b;
			}


		}
		PrismaticJointPrimitive& primitive()
		{
			return m_primitive;
		}
	private:
		PrismaticJointPrimitive m_primitive;
	};
}
#endif