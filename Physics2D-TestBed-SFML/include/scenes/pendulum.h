#ifndef PHYSICS2D_TESTBED_SCENES_PENDULUM_H
#define PHYSICS2D_TESTBED_SCENES_PENDULUM_H

#include "./include/frame.h"
#include <deque>
namespace Physics2D
{
	class PendulumFrame : public Frame
	{
	public:
		PendulumFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Camera* camera) : Frame("Pendulum", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{
			points.resize(400);

			uint32_t mask = 0x01;
			rectangle.set(4.0f, 0.25f);

			stick1 = m_world->createBody();
			stick1->setShape(&rectangle);
			stick1->setMass(2.0f);
			stick1->setBitmask(mask << 1);
			stick1->setType(BodyType::Dynamic);
			stick1->position().set(0, 0);

			stick2 = m_world->createBody();
			stick2->setShape(&rectangle);
			stick2->setMass(2.0f);
			stick2->setBitmask(mask << 2);
			stick2->setType(BodyType::Dynamic);
			stick2->position().set(3.0f, 0);
			stick2->rotation() = 0;

			stick3 = m_world->createBody();
			stick3->setShape(&rectangle);
			stick3->setMass(2.0f);
			stick3->setBitmask(mask << 3);
			stick3->setType(BodyType::Dynamic);
			real h = 1.5f * fastInverseSqrt(2.0f);
			stick3->position().set(3.0f + 1.5f + h, h);
			stick3->rotation() = degreeToRadian(45);

			RevoluteJointPrimitive rjp;
			rjp.bodyA = stick1;
			rjp.bodyB = stick2;
			rjp.localPointA.set(1.5f, 0);
			rjp.localPointB.set(-1.5f, 0);
			rjp.frequency = 10;
			rjp.dampingRatio = 0.8f;
			m_world->createJoint(rjp);

			rjp.bodyA = stick2;
			rjp.bodyB = stick3;
			rjp.localPointA.set(1.5f, 0);
			rjp.localPointB.set(-1.5f, 0);
			rjp.frequency = 10;
			rjp.dampingRatio = 0.8f;
			m_world->createJoint(rjp);

			PointJointPrimitive pjp;
			pjp.bodyA = stick1;
			pjp.frequency = 50;
			pjp.dampingRatio = 0.8f;
			pjp.maxForce = 10000;
			pjp.targetPoint.set(-1.5f, 0);
			pjp.localPointA.set(-1.5f, 0);
			m_world->createJoint(pjp);

			m_tree->insert(stick1);
			m_tree->insert(stick2);
			m_tree->insert(stick3);

			m_world->setEnableDamping(false);
		}
		void render(sf::RenderWindow& window) override
		{
			if (points.size() > 400)
				points.pop_front();
			points.emplace_back(stick3->toWorldPoint(Vec2{ 2.0f, 0.0f }));

			for(auto& elem: points)
				RenderSFMLImpl::renderPoint(window, *m_camera, elem, sf::Color::Cyan);
		}
		void release() override
		{
			m_world->setEnableDamping(true);
		}
	private:
		Body* stick1 = nullptr;
		Body* stick2 = nullptr;
		Body* stick3 = nullptr;
		std::deque<Vec2> points;
		Rectangle rectangle;

	};
}

#endif // !1
