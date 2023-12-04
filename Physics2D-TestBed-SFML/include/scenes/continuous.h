#ifndef PHYSICS2D_SCENES_CONTINUOUS_H
#define PHYSICS2D_SCENES_CONTINUOUS_H
#include "frame.h"

namespace Physics2D
{
	class ContinuousFrame : public Frame
	{
	public:
		ContinuousFrame(const FrameSettings& settings) : Frame(settings)
		{
		}

		void onLoad() override
		{
			block.set(300.0f, 60.0f);
			edge.set({-200, 0}, {149.9f, 0});
			rect.set(1.0f, 1.0f);
			circle.setRadius(0.5f);
			stick.set(2.0f, 0.5f);
			wall.set(60.0, 400.0f);

			Body* ground = m_settings.world->createBody();
			ground->setShape(&block);
			ground->position().set({0, -30});
			ground->setMass(100000);
			ground->setType(Body::BodyType::Static);
			ground->setFriction(0.3f);
			m_settings.tree->insert(ground);

			for (real j = 0; j < 20.0f; j += 1.0f)
			{
				for (real i = 0; i < 3.0; i += 1.0f)
				{
					Body* body = m_settings.world->createBody();
					body->position().set({i * 1.05f - 2.0f, j * 1.05f + 0.55f});
					body->setShape(&rect);
					body->rotation() = 0.0f;
					body->setMass(1.0f);
					body->setType(Body::BodyType::Dynamic);
					body->setFriction(0.5f);
					body->setRestitution(0.0f);
					m_settings.tree->insert(body);
				}
			}

			Body* bullet = m_settings.world->createBody();
			bullet->setShape(&circle);
			bullet->position().set({-100.0f, 8.5f});
			bullet->setType(Body::BodyType::Bullet);
			bullet->setMass(5.0f);
			bullet->velocity().set({1500.0f, 0.0f});
			bullet->angularVelocity() = -500.0f;
			m_settings.tree->insert(bullet);

			Body* wallBody = m_settings.world->createBody();
			wallBody->setShape(&wall);
			wallBody->position().set(180.0f, 0.0f);
			wallBody->setMass(100000);
			wallBody->setFriction(0.1f);
			wallBody->setType(Body::BodyType::Static);

			m_settings.tree->insert(wallBody);

			//m_settings.camera->setTargetBody(bullet);
			//m_settings.camera->setMeterToPixel(5);
		}

		void onPostRender(sf::RenderWindow& window) override
		{
		}

		void onUnLoad() override
		{
			m_settings.camera->setTargetBody(nullptr);
		}

	private:
		Edge edge;
		Rectangle block;
		Rectangle rect;
		Rectangle stick;
		Rectangle wall;
		Circle circle;
	};
}
#endif
