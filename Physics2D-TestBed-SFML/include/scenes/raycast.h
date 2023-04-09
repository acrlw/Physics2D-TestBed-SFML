#ifndef PHYSICS2D_SCENES_RAYCAST_H
#define PHYSICS2D_SCENES_RAYCAST_H
#include <random>
#include "frame.h"

namespace Physics2D
{
	class RaycastFrame : public Frame
	{
	public:
		RaycastFrame(const FrameSettings& settings) : Frame(settings)
		{
		}

		void onLoad() override
		{
			rectangle.set(0.5f, 0.5f);
			circle.setRadius(0.5f);
			capsule.set(1.5f, 0.5f);
			triangle.append({{-1.0f, 1.0f}, {0.0f, -2.0f}, {1.0f, -1.0f}});
			polygon.append({
				{0.0f, 4.0f}, {-3.0f, 3.0f}, {-4.0f, 0.0f}, {-3.0f, -3.0f}, {0, -4.0f},
				{3.0f, -3.0f}, {4.0f, 0.0f}, {3.0f, 3.0f}
			});
			triangle.scale(0.5f);
			polygon.scale(0.1f);


			Shape* shapeArray[5];
			shapeArray[0] = &rectangle;
			shapeArray[1] = &circle;
			shapeArray[2] = &triangle;
			shapeArray[3] = &polygon;
			shapeArray[4] = &capsule;

			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_real_distribution<> dist1(-9.0f, 9.0f);
			std::uniform_int_distribution<> dist2(0, 4);
			std::uniform_real_distribution<> dist3(-Constant::Pi, Constant::Pi);

			for (int i = 0; i < 100; i++)
			{
				Body* body = m_settings.world->createBody();
				body->position().set(dist1(gen), dist1(gen));
				body->setShape(shapeArray[dist2(gen)]);
				body->rotation() = dist3(gen);
				body->setMass(1);
				body->setType(Body::BodyType::Static);

				m_settings.tree->insert(body);
			}
		}

		void onPostRender(sf::RenderWindow& window) override
		{
			Vector2 p;
			Vector2 d = mousePos.normal();
			sf::Color originColor = RenderConstant::Gray;
			sf::Color dirColor = RenderConstant::DarkGreen;
			sf::Color hitColor = sf::Color::Cyan;


			RenderSFMLImpl::renderPoint(window, *m_settings.camera, Vector2(0, 0), originColor);
			RenderSFMLImpl::renderLine(window, *m_settings.camera, p, d * 10.0f, dirColor);
			auto bodyList = m_settings.tree->raycast(p, d);
			for (auto& elem : bodyList)
			{
				ShapePrimitive sp;
				sp.transform.rotation = elem->rotation();
				sp.transform.position = elem->position();
				sp.shape = elem->shape();
				RenderSFMLImpl::renderShape(window, *m_settings.camera, sp, hitColor);
			}
		}

		void onMouseMove(sf::Event& event) override
		{
			mousePos = m_settings.camera->screenToWorld(Vector2(event.mouseMove.x, event.mouseMove.y));
		}

	private:
		Vector2 mousePos = Vector2(1, 1);
		Rectangle rectangle;
		Circle circle;
		Polygon polygon;
		Capsule capsule;
		Polygon triangle;
	};
}
#endif
