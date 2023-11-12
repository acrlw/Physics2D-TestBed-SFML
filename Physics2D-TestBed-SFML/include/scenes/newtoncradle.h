#ifndef PHYSICS2D_TESTBED_SCENES_NEWTONCRADLE_H
#define PHYSICS2D_TESTBED_SCENES_NEWTONCRADLE_H

#include "frame.h"

namespace Physics2D
{
	class NewtonCradleFrame : public Frame
	{
	public:
		NewtonCradleFrame(const FrameSettings& settings) : Frame(settings)
		{
		}

		void onLoad() override
		{
			Edge edge;

			edge.set({ -100, 0 }, { 100, 0 });

			real startX = -5.0f;
			circle.setRadius(1.0f);
			DistanceJointPrimitive djp;
			djp.minDistance = 5.0f;
			djp.maxDistance = 5.0f;
			djp.localPointA.set(0, 0);

			Body* ball = m_settings.world->createBody();
			ball->setShape(&circle);
			ball->setMass(1.0f);
			ball->setType(Body::BodyType::Dynamic);
			ball->position().set(startX, 5.0);
			ball->setRestitution(1.0f);


			Body* ground;

			ground = m_settings.world->createBody();
			ground->setShape(&edge);
			ground->position().set({ 0.0, 0.0 });
			ground->setMass(Constant::Max);
			ground->setType(Body::BodyType::Static);
			ground->setFriction(0.6f);
			m_settings.tree->insert(ground);



			djp.bodyB = ground;
			djp.localPointB.set(startX, 10.0f);

			djp.bodyA = ball;
			
			m_settings.world->createJoint(djp);

			m_settings.tree->insert(ball);


			for (real i = 0; i < 5.0f; i++)
			{
				startX += 2.01f;
				ball = m_settings.world->createBody();
				ball->setShape(&circle);
				ball->setMass(1.0f);
				ball->setType(Body::BodyType::Dynamic);
				ball->setFriction(0.1f);
				ball->setRestitution(1.0f);
				ball->position().set(startX, 5.0);
				djp.localPointB.set(startX, 10.0f);
				djp.bodyA = ball;
				m_settings.world->createJoint(djp);
				m_settings.tree->insert(ball);
			}

			startX += 2.01f;
			ball = m_settings.world->createBody();
			ball->setShape(&circle);
			ball->setMass(1.0f);
			ball->setType(Body::BodyType::Dynamic);
			ball->position().set(startX + 5.0f, 10.0f);
			ball->setRestitution(1.0f);

			djp.localPointB.set(startX, 10.0f);
			djp.bodyA = ball;
			m_settings.world->createJoint(djp);

			m_settings.tree->insert(ball);
		}

		void onPostRender(sf::RenderWindow& window) override
		{
		}

	private:
		Circle circle;
	};
}

#endif // !PHYSICS2D_TESTBED_SCENES_NEWTONCRADLE_H
