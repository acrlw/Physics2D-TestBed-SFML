#ifndef PHYSICS2D_SCENES_WRECKINGBALL_H
#define PHYSICS2D_SCENES_WRECKINGBALL_H
#include "frame.h"

namespace Physics2D
{
	class WreckingBallFrame : public Frame
	{
	public:
		WreckingBallFrame(const FrameSettings& settings) : Frame(settings)
		{
		}

		void onLoad() override
		{
			Body* rect;
			Body* rect2;
			Body* ground;

			rectangle.set(1.0f, 1.0f);
			circle.setRadius(1.5f);
			brick.set(1.5f, 0.5f);
			edge.set({-100, 0}, {100, 0});


			DistanceJointPrimitive distancePrim;


			ground = m_settings.world->createBody();
			ground->setShape(&edge);
			ground->position().set({0, 0.0});
			ground->setMass(Constant::Max);
			ground->setType(Body::BodyType::Static);
			m_settings.tree->insert(ground);

			for (real j = 0; j < 10.0f; j += 1.0f)
			{
				for (real i = 0; i < 1.0; i += 1.0f)
				{
					Body* body = m_settings.world->createBody();
					body->position().set({i * 1.05f - 32.0f, j * 1.05f - ground->position().y + 0.55f});
					body->setShape(&rectangle);
					body->rotation() = 0.0f;
					body->setMass(1.0f);
					body->setType(Body::BodyType::Dynamic);
					body->setFriction(0.1f);
					body->setRestitution(0.0f);
					m_settings.tree->insert(body);
				}
			}
			//for (real j = 0; j < 0.0f; j += 1.0f)
			//{
			//	for (real i = 0; i < 6.0; i += 1.0f)
			//	{
			//		Body* body = m_settings.world->createBody();
			//		body->position().set({i * 1.05f - 0.0f, j * 1.05f - ground->position().y + 0.55f});
			//		body->setShape(&rectangle);
			//		body->rotation() = 0.0f;
			//		body->setMass(1.0f);
			//		body->setType(Body::BodyType::Dynamic);
			//		body->setFriction(0.0f);
			//		body->setRestitution(0.0f);
			//		m_settings.tree->insert(body);
			//	}
			//}

			//real half = brick.width() / 2.0f + 0.1f;
			//rect = m_settings.world->createBody();
			//rect->setShape(&brick);
			//rect->position().set({-20.0f, 20.0f});
			//rect->rotation() = 0;
			//rect->setMass(1.0f);
			//rect->setRestitution(0.2f);
			//rect->setFriction(0.8f);
			//rect->setType(Body::BodyType::Dynamic);


			//RevoluteJointPrimitive ppm;
			//ppm.bodyA = rect;
			//ppm.bodyB = ground;
			//ppm.localPointA.set(-half, 0);
			//ppm.localPointB.set(-20.0f - half, 20.0f);
			//ppm.dampingRatio = 1.0f;
			//ppm.frequency = 10;
			//ppm.angularLimit = false;
			//ppm.maxForce = Constant::Max;
			//m_settings.world->createJoint(ppm);
			//real max = 7.0f;
			//m_settings.tree->insert(rect);
			//for (real i = 1.0f; i < max; i += 1.0f)
			//{
			//	rect2 = m_settings.world->createBody();
			//	rect2->setShape(&brick);
			//	rect2->position().set({-20.0f + i * brick.width() + i * 0.2f, 20.0f});
			//	rect2->rotation() = 0;
			//	rect2->setMass(2.0f);
			//	rect2->setFriction(0.1f);
			//	rect2->setType(Body::BodyType::Dynamic);

			//	m_settings.tree->insert(rect2);
			//	RevoluteJointPrimitive revolutePrim;
			//	revolutePrim.bodyA = rect;
			//	revolutePrim.bodyB = rect2;
			//	revolutePrim.localPointA.set(half, 0);
			//	revolutePrim.localPointB.set(-half, 0);
			//	revolutePrim.dampingRatio = 0.707f;
			//	revolutePrim.frequency = 10;
			//	revolutePrim.maxForce = Constant::Max;
			//	revolutePrim.angularLimit = false;
			//	m_settings.world->createJoint(revolutePrim);
			//	rect = rect2;
			//}
			//rect2 = m_settings.world->createBody();
			//rect2->setShape(&circle);
			//rect2->position().set({-20.0f + max * brick.width() + max * 0.2f + half, 20.0f});
			//rect2->rotation() = 0;
			//rect2->setMass(50.0f);
			//rect2->setFriction(0.1f);
			//rect2->setType(Body::BodyType::Dynamic);

			//m_settings.tree->insert(rect2);
			//RevoluteJointPrimitive revolutePrim;
			//revolutePrim.bodyA = rect;
			//revolutePrim.bodyB = rect2;
			//revolutePrim.localPointA.set(half, 0);
			//revolutePrim.localPointB.set(-half * 2.0f, 0);
			//revolutePrim.dampingRatio = 0.8f;
			//revolutePrim.frequency = 20;
			//revolutePrim.maxForce = Constant::Max;
			//revolutePrim.angularLimit = false;
			//m_settings.world->createJoint(revolutePrim);


			//rect2 = m_settings.world->createBody();
			//rect2->setShape(&circle);
			//rect2->position().set({21.5f + half, 20.0f});
			//rect2->rotation() = 0;
			//rect2->setMass(50.0f);
			//rect2->setFriction(0.1f);
			//rect2->setType(Body::BodyType::Dynamic);
			//m_settings.tree->insert(rect2);

			//distancePrim.bodyA = rect2;
			//distancePrim.bodyB = ground;
			//distancePrim.localPointA.set({0, 0});
			//distancePrim.localPointB.set({ 10, 20 });
			//distancePrim.minDistance = 11.5f + half;
			//distancePrim.maxDistance = 11.5f + half;

			//m_settings.world->createJoint(distancePrim);
		}

	private:
		Rectangle rectangle;
		Rectangle brick;
		Edge edge;
		Circle circle;
	};
}
#endif
