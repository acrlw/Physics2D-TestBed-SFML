#ifndef PHYSICS2D_DYNAMICS_JOINT_WELD_H
#define PHYSICS2D_DYNAMICS_JOINT_WELD_H
#include "physics2d_joint.h"
namespace Physics2D
{
	struct PHYSICS2D_API WeldJointPrimitive
	{
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		Vector2 localPointA;
		Vector2 localPointB;
		real referenceAngle = 0.0f;

		real damping = 0.0f;
		real stiffness = 0.0f;
		real frequency = 2.0f;
		real maxForce = 5000.0f;
		real maxTorque = 5000.0f;
		real dampingRatio = 0.707f;
		real gamma = 0.0f;

		real angularError = 0.0f;
		Vector3 linearError;

		Matrix3x3 invK;
		Vector3 impulse;

		real angularBias = 0.0f;
	};
	//weld joint comes from revolute and rotation joint
	class PHYSICS2D_API WeldJoint : public Joint
	{
	public:
		WeldJoint()
		{
			m_type = JointType::Weld;
		}

		WeldJoint(const WeldJointPrimitive& primitive)
		{
			m_type = JointType::Weld;
			m_primitive = primitive;
		}
		void set(const WeldJointPrimitive& primitive)
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


			if (m_primitive.frequency > 0.0)
			{
				real nf = naturalFrequency(m_primitive.frequency);
				m_primitive.stiffness = springStiffness(m_a + m_b, nf);
				m_primitive.damping = springDampingCoefficient(m_a + m_b, nf, m_primitive.dampingRatio);
			}
			else
			{
				m_primitive.stiffness = 0.0;
				m_primitive.damping = 0.0;
			}
			m_primitive.gamma = constraintImpulseMixing(dt, m_primitive.stiffness, m_primitive.damping);
			real erp = errorReductionParameter(dt, m_primitive.stiffness, m_primitive.damping);

			Vector2 pa = bodyA->toWorldPoint(m_primitive.localPointA);
			Vector2 ra = pa - bodyA->position();
			Vector2 pb = bodyB->toWorldPoint(m_primitive.localPointB);
			Vector2 rb = pb - bodyB->position();

			Matrix3x3 k;
			k.e11() = im_a + im_b + ra.y * ra.y * ii_a + rb.y * rb.y * ii_b;
			k.e21() = -ra.y * ra.x * ii_a - rb.y * rb.x * ii_b;
			k.e31() = -ra.y * ii_a - rb.y * ii_b;
			k.e12() = k.e21();
			k.e22() = im_a + im_b + ra.x * ra.x * ii_a + rb.x * rb.x * ii_b;
			k.e23() = ra.x * ii_a + rb.x * ii_b;
			k.e31() = k.e31();
			k.e32() = k.e23();
			k.e33() = ii_a + ii_b;

			Matrix3x3 JMJt = k;

			//soften angular constraint
			if(m_primitive.stiffness > 0.0f)
			{
				Matrix2x2 JMJt1(k.e11(), k.e21(), k.e12(), k.e22());
				JMJt1.invert();
				m_primitive.invK = JMJt1;

				real c = bodyA->rotation() - bodyB->rotation() - m_primitive.referenceAngle;

				real diffAngle = Math::radianToDegree(c);

				real error = (pa - pb).length();

				real inv_I = ii_a + ii_b;

				m_primitive.angularBias = erp * c;

				inv_I += m_primitive.gamma;
				if(inv_I != 0.0f)
					m_primitive.invK.e33() = 1.0f / inv_I;

			}
			else
			{

				k.invert();
				m_primitive.invK = k;
			}

			//hard constraint
			real c = bodyA->rotation() - bodyB->rotation() - m_primitive.referenceAngle;

			real diffAngle = Math::radianToDegree(c);
			k.invert();
			m_primitive.invK = k;

			m_primitive.bodyA->angularVelocity() += m_primitive.impulse.z * ii_a;
			m_primitive.bodyB->angularVelocity() -= m_primitive.impulse.z * ii_b;

			Vector2 impulse(m_primitive.impulse.x, m_primitive.impulse.y);
			m_primitive.bodyA->applyImpulse(impulse, ra);
			m_primitive.bodyB->applyImpulse(-impulse, rb);
			
		}
		void solveVelocity(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;

			Body* bodyA = m_primitive.bodyA;
			Body* bodyB = m_primitive.bodyB;

			real ii_a = bodyA->inverseInertia();
			real ii_b = bodyB->inverseInertia();

			Vector2 ra = bodyA->toWorldPoint(m_primitive.localPointA) - bodyA->position();
			real wa = bodyA->angularVelocity();

			Vector2 rb = bodyB->toWorldPoint(m_primitive.localPointB) - bodyB->position();
			real wb = bodyB->angularVelocity();

			//soft constraint
			if(m_primitive.stiffness > 0.0f)
			{
				real dw = wa - wb;

				real torque = -m_primitive.invK.e33() * (dw + m_primitive.gamma * m_primitive.impulse.z + m_primitive.angularBias);
				m_primitive.impulse.z += torque;

				bodyA->angularVelocity() += ii_a * torque;
				bodyB->angularVelocity() -= ii_b * torque;

				Vector2 va = bodyA->velocity() + Vector2::crossProduct(bodyA->angularVelocity(), ra);
				Vector2 vb = bodyB->velocity() + Vector2::crossProduct(bodyB->angularVelocity(), rb);

				Vector2 dv = va - vb;
				Vector2 impulse = -m_primitive.invK.multiply(dv);
				m_primitive.impulse.x += impulse.x;
				m_primitive.impulse.y += impulse.y;

				bodyA->applyImpulse(impulse, ra);
				bodyB->applyImpulse(-impulse, rb);

			}

			//hard constraint
			//real dw = wa - wb;
			//Vector2 va = bodyA->velocity() + Vector2::crossProduct(bodyA->angularVelocity(), ra);
			//Vector2 vb = bodyB->velocity() + Vector2::crossProduct(bodyB->angularVelocity(), rb);

			//Vector2 dv = va - vb;
			//Vector3 jv(dv.x, dv.y, dw);
			//Vector3 x = -m_primitive.invK.multiply(jv);

			//real t = x.z;
			//Vector2 P(x.x, x.y);
			//bodyA->angularVelocity() += ii_a * t;
			//bodyB->angularVelocity() -= ii_b * t;

			//bodyA->applyImpulse(P, ra);
			//bodyB->applyImpulse(-P, rb);


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
			Vector2 ra = pa - bodyA->position();
			Vector2 pb = bodyB->toWorldPoint(m_primitive.localPointB);
			Vector2 rb = pb - bodyB->position();

			Matrix3x3 k;
			k.e11() = im_a + im_b + ra.y * ra.y * ii_a + rb.y * rb.y * ii_b;
			k.e21() = -ra.y * ra.x * ii_a - rb.y * rb.x * ii_b;
			k.e31() = -ra.y * ii_a - rb.y * ii_b;
			k.e12() = k.e21();
			k.e22() = im_a + im_b + ra.x * ra.x * ii_a + rb.x * rb.x * ii_b;
			k.e23() = ra.x * ii_a + rb.x * ii_b;
			k.e31() = k.e13();
			k.e32() = k.e23();
			k.e33() = ii_a + ii_b;

			if(m_primitive.stiffness > 0.0f)
			{
				Vector2 c = pa - pb;
				real error = (pa - pb).length();

				Matrix2x2 JMJt1(k.e11(), k.e21(), k.e12(), k.e22());
				JMJt1.invert();
				Vector2 P = -JMJt1.multiply(c);

				bodyA->position() += im_a * P;
				bodyA->rotation() += ii_a * Vector2::crossProduct(ra, P);

				bodyB->position() -= im_b * P;
				bodyB->rotation() -= ii_b * Vector2::crossProduct(rb, P);

			}
			else
			{
				k.invert();
				Vector2 c = pa - pb;
				real a = bodyA->rotation() - bodyB->rotation() - m_primitive.referenceAngle;
				Vector3 C(c.x, c.y, a);

				Vector3 impulse = -k.multiply(C);
				Vector2 lambda1 = Vector2(impulse.x, impulse.y);
				real lambda2 = impulse.z;

				Vector2 delta_p = 0.0f;
				real delta_o = 0.0f;

				bodyA->position() += im_a * lambda1;
				bodyA->rotation() += ii_a * Vector2::crossProduct(ra, lambda1) + ii_a * lambda2;

				delta_p += im_a * lambda1;
				delta_o += Vector2::crossProduct(ra, lambda1) * ii_a + lambda2 * ii_a;

				bodyB->position() += -im_b * lambda1;
				bodyB->rotation() += -ii_b * Vector2::crossProduct(rb, lambda1) - ii_b * lambda2;

				delta_p += -im_b * lambda1;
				delta_o += -ii_b * Vector2::crossProduct(rb, lambda1) - ii_b * lambda2;


				std::cout << "c:" << (pa - pb).length() << std::endl;
				std::cout << "length of delta_p: " << delta_p.length() << std::endl;
				std::cout << "delta_p: " << delta_p.x << "," << delta_p.y << std::endl;
				std::cout << "sum of delta_o: " << delta_o << std::endl;

				m_primitive.invK = k;
			}

		}
		real accumulatedImpulse() override
		{
			Vector2 lambda(m_primitive.impulse.x, m_primitive.impulse.y);
			return lambda.length();
		}
		Vector2 jacobian() override
		{
			Vector2 lambda(m_primitive.impulse.x, m_primitive.impulse.y);
			return lambda.normal();
		}
		WeldJointPrimitive primitive()const
		{
			return m_primitive;
		}
	private:
		WeldJointPrimitive m_primitive;
	};
}
#endif