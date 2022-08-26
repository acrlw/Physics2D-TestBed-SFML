#ifndef PHYSICS2D_DYNAMICS_JOINT_REVOLUTE_H
#define PHYSICS2D_DYNAMICS_JOINT_REVOLUTE_H

namespace Physics2D
{
	struct RevoluteJointPrimitive
	{
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		Vec2 localPointA;
		Vec2 localPointB;

		real damping = 0.0f;
		real stiffness = 0.0f;
		real frequency = 8.0f;
		real maxForce = 5000.0f;
		real dampingRatio = 0.2f;
		real gamma = 0.0f;
		Vec2 bias;
		Mat2 effectiveMass;
		Vec2 accumulatedImpulse;
	};
	class RevoluteJoint : public Joint
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

			Vec2 pa = bodyA->toWorldPoint(m_primitive.localPointA);
			Vec2 ra = pa - bodyA->position();
			Vec2 pb = bodyB->toWorldPoint(m_primitive.localPointB);
			Vec2 rb = pb - bodyB->position();

			m_primitive.bias = (pa - pb) * erp;
			Mat2 k;

			k[0][0] = im_a + ra.y * ra.y * ii_a + im_b + rb.y * rb.y * ii_b + m_primitive.gamma;
			k[1][0] = -ra.x * ra.y * ii_a - rb.x * rb.y * ii_b;
			k[0][1] = -ra.x * ra.y * ii_a - rb.x * rb.y * ii_b;
			k[1][1] = im_a + ra.x * ra.x * ii_a + im_b + rb.x * rb.x * ii_b + m_primitive.gamma;


			m_primitive.effectiveMass = k.invert();
			m_primitive.bodyA->applyImpulse(m_primitive.accumulatedImpulse, ra);
			m_primitive.bodyB->applyImpulse(-m_primitive.accumulatedImpulse, rb);

		}
		void solveVelocity(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;

			Vec2 ra = m_primitive.bodyA->toWorldPoint(m_primitive.localPointA) - m_primitive.bodyA->position();
			Vec2 va = m_primitive.bodyA->velocity() + Vec2::crossProduct(m_primitive.bodyA->angularVelocity(), ra);
			Vec2 rb = m_primitive.bodyB->toWorldPoint(m_primitive.localPointB) - m_primitive.bodyB->position();
			Vec2 vb = m_primitive.bodyB->velocity() + Vec2::crossProduct(m_primitive.bodyB->angularVelocity(), rb);

			Vec2 jvb = va - vb;
			jvb += m_primitive.bias;
			jvb += m_primitive.accumulatedImpulse * m_primitive.gamma;
			jvb.negate();
			Vec2 J = m_primitive.effectiveMass.multiply(jvb);
			Vec2 oldImpulse = m_primitive.accumulatedImpulse;
			m_primitive.accumulatedImpulse += J;
			real maxImpulse = dt * m_primitive.maxForce;
			if (m_primitive.accumulatedImpulse.magnitudeSquare() > maxImpulse * maxImpulse)
			{
				m_primitive.accumulatedImpulse.normalize();
				m_primitive.accumulatedImpulse *= maxImpulse;
			}
			J = m_primitive.accumulatedImpulse - oldImpulse;
			m_primitive.bodyA->applyImpulse(J, ra);
			m_primitive.bodyB->applyImpulse(-J, rb);

		}
		void solvePosition(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr || m_primitive.bodyB == nullptr)
				return;
			//Body* bodyA = m_primitive.bodyA;
			//Body* bodyB = m_primitive.bodyB;
			//Vec2 pa = bodyA->toWorldPoint(m_primitive.localPointA);
			//Vec2 ra = pa - bodyA->position();
			//Vec2 pb = bodyB->toWorldPoint(m_primitive.localPointB);
			//Vec2 rb = pb - bodyB->position();

			//Vec2 bias = (pa - pb) * 0.001f;
			//Vec2 impulse = m_primitive.effectiveMass.multiply(bias);

			//if (bodyA->type() != BodyType::Static && !bodyA->sleep())
			//{
			//	bodyA->position() += bodyA->inverseMass() * impulse;
			//	bodyA->rotation() += bodyA->inverseInertia() * ra.cross(impulse);
			//}
			//if (bodyB->type() != BodyType::Static && !bodyB->sleep())
			//{
			//	bodyB->position() -= bodyB->inverseMass() * impulse;
			//	bodyB->rotation() -= bodyB->inverseInertia() * rb.cross(impulse);
			//}
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