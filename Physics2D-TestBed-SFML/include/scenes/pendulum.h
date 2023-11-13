#ifndef PHYSICS2D_TESTBED_SCENES_PENDULUM_H
#define PHYSICS2D_TESTBED_SCENES_PENDULUM_H

#include "frame.h"
#include <deque>

namespace Physics2D
{
	class PendulumFrame : public Frame
	{
	public:
		PendulumFrame(const FrameSettings& settings) : Frame(settings)
		{
		}

		void onLoad() override
		{
			Edge edge;
			edge.set(Vector2{ -100.0f, 0 }, Vector2{ 100.0f, 0 });

			points.resize(400);

			uint32_t mask = 0x01;
			rectangle.set(4.0f, 0.25f);

			stick1 = m_settings.world->createBody();
			stick1->setShape(&rectangle);
			stick1->setMass(2.0f);
			stick1->setBitmask(mask << 1);
			stick1->setType(Body::BodyType::Dynamic);
			stick1->position().set(0, 0);

			stick2 = m_settings.world->createBody();
			stick2->setShape(&rectangle);
			stick2->setMass(2.0f);
			stick2->setBitmask(mask << 2);
			stick2->setType(Body::BodyType::Dynamic);
			stick2->position().set(3.0f, 0);
			stick2->rotation() = 0;

			stick3 = m_settings.world->createBody();
			stick3->setShape(&rectangle);
			stick3->setMass(2.0f);
			stick3->setBitmask(mask << 3);
			stick3->setType(Body::BodyType::Dynamic);
			real h = 1.5f * Math::fastInverseSqrt(2.0f);
			stick3->position().set(3.0f + 1.5f + h, h);
			stick3->rotation() = Math::degreeToRadian(45);

			RevoluteJointPrimitive rjp;
			rjp.angularLimit = false;
			rjp.bodyA = stick1;
			rjp.bodyB = stick2;
			rjp.localPointA.set(1.5f, 0);
			rjp.localPointB.set(-1.5f, 0);
			rjp.frequency = 10;
			rjp.dampingRatio = 0.8f;
			m_settings.world->createJoint(rjp);

			rjp.bodyA = stick2;
			rjp.bodyB = stick3;
			rjp.localPointA.set(1.5f, 0);
			rjp.localPointB.set(-1.5f, 0);
			rjp.frequency = 10;
			rjp.dampingRatio = 0.8f;
			m_settings.world->createJoint(rjp);

			Body* ground = m_settings.world->createBody();
			ground->setShape(&edge);
			ground->setType(Body::BodyType::Static);
			ground->setMass(Constant::Max);
			ground->position().set({ 0.0f, 0.0f });
			ground->rotation() = 0.0f;
			ground->setFriction(0.1f);
			ground->setRestitution(0.0f);
			ground->setBitmask(mask << 4);
			
			rjp.bodyA = stick1;
			rjp.bodyB = ground;
			rjp.frequency = 50;
			rjp.dampingRatio = 0.8f;
			rjp.maxForce = 10000;
			rjp.localPointA.set(-1.5f, 0);
			rjp.localPointB.set(-1.5f, 0.0);
			m_settings.world->createJoint(rjp);

			m_settings.tree->insert(stick1);
			m_settings.tree->insert(stick2);
			m_settings.tree->insert(stick3);
			m_settings.tree->insert(ground);

			m_settings.world->setEnableDamping(false);
		}

		void onPostRender(sf::RenderWindow& window) override
		{
			if (points.size() > 400)
				points.pop_front();
			points.emplace_back(stick3->toWorldPoint(Vector2{2.0f, 0.0f}));

			Container::Vector<sf::Vertex> vertices;
			vertices.reserve(points.size());
			for (auto& elem : points)
			{
				Vector2 screenPos = m_settings.camera->worldToScreen(elem);
				sf::Vertex vertex;
				vertex.position = RenderSFMLImpl::toVector2f(screenPos);
				vertex.color = sf::Color::Cyan;
				vertices.emplace_back(vertex);
			}
			window.draw(&vertices[0], vertices.size(), sf::Points);
		}

		void onUnLoad() override
		{
			m_settings.world->setEnableDamping(true);
		}

	private:
		Body* stick1 = nullptr;
		Body* stick2 = nullptr;
		Body* stick3 = nullptr;
		std::deque<Vector2> points;
		Rectangle rectangle;
	};
}

#endif // !1
