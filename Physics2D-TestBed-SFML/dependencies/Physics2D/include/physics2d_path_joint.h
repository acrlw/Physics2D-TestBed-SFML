#ifndef PHYSICS2D_DYNAMICS_JOINT_PATH_H
#define PHYSICS2D_DYNAMICS_JOINT_PATH_H
#include "physics2d_joint.h"

namespace Physics2D
{
	struct PHYSICS2D_API PathJointPrimitive
	{
		//for example, circle
		Body* bodyA = nullptr;
		Vector2 localPointA;

		Vector2 closestPoint;
		Vector2 origin;
		real radius = 2.0f;

		Vector2 normal;
		Vector2 tangent;

		Matrix2x2 invK;

		Vector2 impulse;
	};

	class PHYSICS2D_API PathJoint : public Joint
	{
	public:
		PathJoint()
		{
			m_type = JointType::Path;
		}

		PathJoint(const PathJointPrimitive& prim) : m_primitive(prim)
		{
			m_type = JointType::Path;
		}

		void set(const PathJointPrimitive& prim)
		{
			m_primitive = prim;
		}

		void prepare(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr)
				return;
			Body* bodyA = m_primitive.bodyA;
			
			real im_a = bodyA->inverseMass();
			real ii_a = bodyA->inverseInertia();

			Vector2 pa = bodyA->toWorldPoint(m_primitive.localPointA);
			Vector2 ra = pa - bodyA->position();

			Vector2 ba = pa - m_primitive.origin;
			m_primitive.normal = ba.normal();
			m_primitive.closestPoint = m_primitive.origin + m_primitive.normal * m_primitive.radius;

			Matrix2x2& k = m_primitive.invK;
			k.e11() = im_a + ii_a * ra.cross(m_primitive.normal) * ra.cross(m_primitive.normal);
			k.e12() = ii_a * ra.cross(m_primitive.normal);
			k.e21() = ii_a * ra.cross(m_primitive.normal);
			k.e22() = ii_a;

			k.invert();

			Vector2 P = m_primitive.impulse.x * m_primitive.normal;
			bodyA->applyImpulse(P, ra);
			bodyA->angularVelocity() += ii_a * m_primitive.impulse.y;

		}

		void solveVelocity(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr)
				return;

			Body* bodyA = m_primitive.bodyA;
			Vector2 pa = bodyA->toWorldPoint(m_primitive.localPointA);
			Vector2 ra = pa - bodyA->position();
			Vector2 va = bodyA->velocity() + Vector2::crossProduct(bodyA->angularVelocity(), ra);

			real nv = m_primitive.normal.dot(va);
			real dw = bodyA->angularVelocity();

			Vector2 dv(nv, dw);
			Vector2 lambda = m_primitive.invK.multiply(-dv);
			m_primitive.impulse += lambda;
			Vector2 P1 = m_primitive.normal * lambda.x;

			bodyA->angularVelocity() += bodyA->inverseInertia() * lambda.y;

			bodyA->applyImpulse(P1, ra);

		}

		void solvePosition(const real& dt) override
		{
			if (m_primitive.bodyA == nullptr)
				return;
			Body* bodyA = m_primitive.bodyA;
			
			real im_a = bodyA->inverseMass();
			real ii_a = bodyA->inverseInertia();

			Vector2 pa = bodyA->toWorldPoint(m_primitive.localPointA);
			Vector2 ra = pa - bodyA->position();

			Vector2 ba = pa - m_primitive.origin;
			m_primitive.normal = ba.normal();
			m_primitive.tangent = m_primitive.normal.perpendicular();
			m_primitive.closestPoint = m_primitive.origin + m_primitive.normal * m_primitive.radius;

			Vector2 c = pa - m_primitive.closestPoint;
			real linearError = c.dot(m_primitive.normal);

			Vector2 localB = bodyA->toLocalPoint(bodyA->position() + m_primitive.tangent);
			real angularError = -localB.theta();

			Matrix2x2 k;
			k.e11() = im_a + ii_a * ra.cross(m_primitive.normal) * ra.cross(m_primitive.normal);
			k.e12() = ii_a * ra.cross(m_primitive.normal);
			k.e21() = ii_a * ra.cross(m_primitive.normal);
			k.e22() = ii_a;

			k.invert();

			Vector2 dv(linearError, angularError);

			Vector2 lambda = k.multiply(-dv);

			Vector2 P1 = m_primitive.normal * lambda.x;

			bodyA->rotation() += bodyA->inverseInertia() * lambda.y;

			bodyA->position() += P1 * im_a;

		}

		PathJointPrimitive& primitive()
		{
			return m_primitive;
		}

	private:
		PathJointPrimitive m_primitive;
		real m_factor = 0.22f;
	};
}
#endif
