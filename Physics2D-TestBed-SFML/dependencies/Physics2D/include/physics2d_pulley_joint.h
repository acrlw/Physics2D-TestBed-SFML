#ifndef PHYSICS2D_DYNAMICS_JOINT_PULLEY_H
#define PHYSICS2D_DYNAMICS_JOINT_PULLEY_H
#include "physics2d_joint.h"
namespace Physics2D
{
	struct PHYSICS2D_API PulleyJointPrimitive
	{
		Body* bodyA = nullptr;
		Body* bodyB = nullptr;
		real ratio = 1.0f;
		Vector2 localPointA;
		Vector2 localPointB;
		Vector2 targetPointA;
		Vector2 targetPointB;
	};
	class PHYSICS2D_API PulleyJoint : public Joint
	{
	public:
		PulleyJoint()
		{
			m_type = JointType::Pulley;
		}

		PulleyJoint(const PulleyJointPrimitive& primitive)
		{
			m_type = JointType::Pulley;
			m_primitive = primitive;
		}
		void set(const PulleyJointPrimitive& primitive)
		{
			m_primitive = primitive;
		}
		void prepare(const real& dt) override
		{

		}
		void solveVelocity(const real& dt) override
		{

		}
		void solvePosition(const real& dt) override
		{

		}
	private:
		PulleyJointPrimitive m_primitive;
	};
}
#endif