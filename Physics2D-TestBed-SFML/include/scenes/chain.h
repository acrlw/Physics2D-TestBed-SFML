#ifndef PHYSICS2D_SCENES_CHAIN_H
#define PHYSICS2D_SCENES_CHAIN_H
#include "frame.h"

namespace Physics2D
{
	class ChainFrame : public Frame
	{
	public:
		ChainFrame(const FrameSettings& settings) : Frame(settings)
		{
		}

		void onLoad() override
		{
			brick.set(1.5f, 0.5f);

			edge.set({-100, 0}, {100, 0});

			Body* rect;
			Body* rect2;
			Body* ground;
			const real offset = 0.75f;
			real half = brick.width() / 2.0f;
			rect = m_settings.world->createBody();
			rect->setShape(&brick);
			rect->position().set({offset, 0.0f});
			rect->rotation() = 0;
			rect->setMass(1.0f);
			rect->setRestitution(0.2f);
			rect->setFriction(0.01f);
			rect->setType(Body::BodyType::Dynamic);

			ground = m_settings.world->createBody();
			ground->setShape(&edge);
			ground->position().set({0, -15.0});
			ground->setMass(Constant::Max);
			ground->setType(Body::BodyType::Static);
			m_settings.tree->insert(ground);

			PointJointPrimitive ppm;
			RevoluteJointPrimitive revolutePrim;

			ppm.bodyA = rect;
			ppm.localPointA.set(-half, 0);
			ppm.targetPoint.set(offset - half, 0.0f);
			ppm.dampingRatio = 0.8f;
			ppm.frequency = 1000;
			ppm.maxForce = 10000;
			m_settings.world->createJoint(ppm);
			real max = 9.0f;
			m_settings.tree->insert(rect);
			for (real i = 1.0f; i < max; i += 1.0f)
			{
				rect2 = m_settings.world->createBody();
				rect2->setShape(&brick);
				rect2->position().set({offset + i * brick.width() * 1.4f, 0.0f});
				rect2->rotation() = 0;
				rect2->setMass(1.0f);
				rect2->setFriction(0.01f);
				rect2->setType(Body::BodyType::Dynamic);

				m_settings.tree->insert(rect2);
				revolutePrim.bodyA = rect;
				revolutePrim.bodyB = rect2;
				revolutePrim.localPointA.set(half + brick.width() * 0.2f, 0);
				revolutePrim.localPointB.set(-half - brick.width() * 0.2f, 0);
				revolutePrim.dampingRatio = 0.8f;
				revolutePrim.frequency = 10;
				revolutePrim.maxForce = 10000;
				m_settings.world->createJoint(revolutePrim);

				rect = rect2;
			}
		}

		void onPostRender(sf::RenderWindow& window) override
		{
		}

	private:
		Rectangle brick;
		Edge edge;
	};
}
#endif
