#ifndef PHYSICS2D_SCENES_COLLISION_H
#define PHYSICS2D_SCENES_COLLISION_H
#include "frame.h"

namespace Physics2D
{
	class CollisionFrame : public Frame
	{
	public:
		CollisionFrame(const FrameSettings& settings) : Frame(settings)
		{
		}

		void onLoad() override
		{
			smallBrick.set(1.0f, 1.0f);
			triangle.append({{1.0f, 0.0f}, {0.0f, 1.0f}, {-1.0f, 0.0f}});
			edge.set({-50, 0}, {50, 0});
			capsule.set(2.0f, 1.0f);

			rectangle.set(1.0f, 1.0f);

			for (real j = 0; j < 1.0f; j += 1.0f)
			{
				for (real i = 0; i < 1.0; i += 1.0f)
				{
					Body* body = m_settings.world->createBody();
					body->position().set({ i * 1.05f + 0.0f, j * 1.05f - 0.0f + 2.25f });
					body->setShape(&rectangle);
					body->rotation() = 0.0f;
					body->setMass(1.0f);
					body->setType(Body::BodyType::Dynamic);
					body->setFriction(0.5f);
					body->setRestitution(0.0f);
					m_settings.tree->insert(body);
				}
			}

			ground = m_settings.world->createBody();
			ground->setShape(&edge);
			ground->position().set({0, 0});
			ground->setMass(Constant::Max);
			ground->setType(Body::BodyType::Static);
			ground->setFriction(0.6f);
			ground->setRestitution(0.0);
			m_settings.tree->insert(ground);
			

			//rect = m_settings.world->createBody();
			//rect->setShape(&smallBrick);
			//rect->position().set({ -0.5f, 0.5f });
			//rect->rotation() = Math::degreeToRadian(0);
			//rect->setMass(1);
			//rect->setType(Body::BodyType::Dynamic);
			//rect->setFriction(0.4f);
			//rect->setRestitution(0.0f);
			//m_settings.tree->insert(rect);

			//rect = m_settings.world->createBody();
			//rect->setShape(&smallBrick);
			//rect->position().set({ 0.5f, 0.5f });
			//rect->rotation() = Math::degreeToRadian(0);
			//rect->setMass(1);
			//rect->setType(Body::BodyType::Dynamic);
			//rect->setFriction(0.4f);
			//rect->setRestitution(0.0f);
			//m_settings.tree->insert(rect);
		}


		void onPostRender(sf::RenderWindow& window) override
		{

			int a = 0;
		}

	private:
		Rectangle smallBrick;
		Body* ground;
		Body* rect;
		Capsule capsule;
		Edge edge;

		Rectangle rectangle;
		Polygon triangle;

	};
}
#endif
