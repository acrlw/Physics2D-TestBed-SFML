#ifndef PHYSICS2D_SCENES_CHAIN_H
#define PHYSICS2D_SCENES_CHAIN_H
#include "frame.h"
#include <random>
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
			circle.setRadius(0.5f);
			edge.set({-100, 0}, {100, 0});

			Body* rect;
			Body* rect2;
			Body* ground;
			const real offset = 0.75f;
			real half = brick.width() / 2.0f;

			ground = m_settings.world->createBody();
			ground->setShape(&edge);
			ground->position().set({ 0, 50.0 });
			ground->setMass(Constant::Max);
			ground->setType(Body::BodyType::Static);
			m_settings.tree->insert(ground);

			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_real_distribution<> pos(-10.0f, 10.0f);
			std::uniform_real_distribution<> rad(0, Constant::Pi * 2);

			rect = m_settings.world->createBody();
			rect->setShape(&circle);
			//rect->position().set( pos(gen), pos(gen));
			rect->position().set( 5.0f, 5.0f);
			rect->rotation() = rad(gen);
			//rect->rotation() = Math::degreeToRadian(-45);
			rect->setMass(2.0f);
			rect->setRestitution(0.2f);
			rect->setFriction(0.01f);
			rect->setType(Body::BodyType::Dynamic);



			WeldJointPrimitive ppm;
			WeldJointPrimitive revolutePrim;

			ppm.bodyA = rect;
			ppm.bodyB = ground;
			ppm.localPointA.set(0.0f, circle.radius());
			ppm.localPointB.set(0.0f, -circle.radius() - 50.0f);
			ppm.dampingRatio = 0.8f;
			ppm.frequency = 0;
			ppm.maxForce = 10000;
			//m_settings.world->createJoint(ppm);
			real max = 2.0;
			m_settings.tree->insert(rect);
			for (real i = 1.0f; i < max; i += 1.0f)
			{
				rect2 = m_settings.world->createBody();
				rect2->setShape(&circle);
				rect2->position().set(-5.0f, -5.0f);
				rect2->rotation() = rad(gen);
				//rect2->rotation() = Math::degreeToRadian(-45);
				rect2->setMass(1.0f);
				rect2->setFriction(0.01f);
				rect2->setType(Body::BodyType::Dynamic);

				m_settings.tree->insert(rect2);
				revolutePrim.bodyA = rect;
				revolutePrim.bodyB = rect2;
				revolutePrim.localPointA.set(0.0, -circle.radius());
				revolutePrim.localPointB.set(0.0f, circle.radius());
				revolutePrim.dampingRatio = 0.8f;
				revolutePrim.frequency = 0;
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
		Circle circle;
	};
}
#endif
