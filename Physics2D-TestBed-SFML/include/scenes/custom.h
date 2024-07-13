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
			real max = 2.0f;

			mask = 0x01;
			Vector2 pos[3];
			pos[0].set({ 0.0f, circle.radius() * Math::sqrt(4.0f - 1.5f * 1.5f)});
			pos[1].set({ -1.5f * circle.radius(), 0.0f});
			pos[2].set({ 1.5f * circle.radius(), 0.0f });
			//pos[0].set({ 0.0f, 0.5f });
			//pos[1].set({ 0.0f, 3.0f * 0.5f });
			//pos[2].set({ 0.0f, 4.0f * 0.5f + circle.radius() });
			for (real i = 0; i < 3; i += 1.0f)
			{
				Body* body = m_settings.world->createBody();
				body->setShape(&circle);
				body->position().set(pos[int(i)]);
				body->setFriction(1.0f);
				body->setBitmask(mask);
				body->setRestitution(0);
				body->setMass(1.0f);
				body->setType(Body::BodyType::Dynamic);
				//mask = mask << 1;
				m_settings.tree->insert(body);
			}
			for (real i = 0; i < 1.0f; i += 1.0f)
			{
				Body* ground = m_settings.world->createBody();
				ground->setShape(&edge);
				ground->position().set({ 0, -circle.radius() });
				ground->setFriction(1.0f);
				ground->setBitmask(mask);
				ground->setRestitution(0);
				ground->setMass(Constant::Max);
				ground->setType(Body::BodyType::Static);
				mask = mask << 1;
				m_settings.tree->insert(ground);
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
