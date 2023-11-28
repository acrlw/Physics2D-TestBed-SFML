#ifndef PHYSICS2D_SCENES_CUSTOM_H
#define PHYSICS2D_SCENES_CUSTOM_H
#include "frame.h"

namespace Physics2D
{
	class CustomFrame : public Frame
	{
	public:
		CustomFrame(const FrameSettings& settings) : Frame(settings)
		{
		}

		void onLoad() override
		{
			block.set(20, 1.0f);
			rectangle.set(1.0f, 1.0f);
			edge.set(Vector2{-100.0f, 0.0f}, Vector2{100.0f, 0.0f});
			circle.setRadius(0.5f);

			uint32_t mask = 0x01;
			real max = 15.0f;
			for (real i = 0; i < 1.0f; i += 1.0f)
			{
				Body* ground = m_settings.world->createBody();
				ground->setShape(&edge);
				ground->position().set({0, 0 + i * 3.0f});
				ground->setFriction(0.4f);
				ground->setBitmask(mask);
				ground->setRestitution(0);
				ground->setMass(Constant::Max);
				ground->setType(Body::BodyType::Static);
				mask = mask << 1;
				m_settings.tree->insert(ground);
			}
			mask = 0x01;
			for (real i = 0; i < max; i += 1.0f)
			{
				Body* body = m_settings.world->createBody();
				body->setShape(&circle);
				body->position().set({ 0.5f, 0.55f + i * 1.05f});
				body->setFriction(0.9f);
				body->setBitmask(mask);
				body->setRestitution(0);
				body->setMass(1);
				body->setType(Body::BodyType::Dynamic);
				//mask = mask << 1;
				m_settings.tree->insert(body);
			}
			//m_settings.world->setEnableDamping(false);
		}

		void onPostRender(sf::RenderWindow& window) override
		{
		}

		void onUnLoad() override
		{
			//m_settings.world->setEnableDamping(true);
		}
	private:
		Rectangle rectangle;
		Rectangle block;
		Edge edge;
		Circle circle;
	};
}
#endif
