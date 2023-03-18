#ifndef PHYSICS2D_SCENES_RAYCAST_H
#define PHYSICS2D_SCENES_RAYCAST_H
#include <random>
#include "frame.h"
namespace Physics2D
{
	class RaycastFrame : public Frame
	{
	public:
		RaycastFrame(PhysicsSystem* system, Camera* camera) : Frame("Raycast", system, camera)
		{

		}
		void load() override
		{
			rectangle.set(0.5f, 0.5f);
			circle.setRadius(0.5f);
			capsule.set(1.5f, 0.5f);
			triangle.append({ {-1.0f, 1.0f},{0.0f, -2.0f},{1.0f, -1.0f},{-1.0f, 1.0f} });
			polygon.append({ {0.0f, 4.0f},{-3.0f, 3.0f},{-4.0f, 0.0f},{-3.0f, -3.0f},{0, -4.0f},
			{3.0f, -3.0f}, {4.0f, 0.0f }, {3.0f, 3.0f },{0.0f, 4.0f } });
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

			//for(int i = 0;i < 100; i++)
			//{
			//	Body* body = m_world->createBody();
			//	body->position().set(dist1(gen), dist1(gen));
			//	body->setShape(shapeArray[dist2(gen)]);
			//	body->rotation() = dist3(gen);
			//	body->setMass(1);
			//	body->setType(BodyType::Static);

			//	m_tree->insert(body);
			//}

		}
		void render(sf::RenderWindow& window) override
		{
			Vec2 p;
			Vec2 d = m_currentPos.normal();
			sf::Color originColor = RenderConstant::MaterialGray;
			sf::Color dirColor = RenderConstant::MaterialDarkGreen;
			sf::Color hitColor = sf::Color::Cyan;


			RenderSFMLImpl::renderPoint(window, *m_camera, Vec2(0, 0), originColor);
			RenderSFMLImpl::renderLine(window, *m_camera, p, d * 10.0f, dirColor);
			m_currentPos = Vec2::lerp(m_currentPos, m_mousePos, 0.1f);
			//auto bodyList = m_tree->raycast(p, d);
			//for(auto& elem: bodyList)
			//{
			//	ShapeTransform sp;
			//	sp.rotation = elem->rotation();
			//	sp.transform = elem->position();
			//	sp.shape = elem->shape();
			//	RenderSFMLImpl::renderShape(window, *m_camera, sp, hitColor);
			//}

		}
		void onMouseMove(sf::Event& event) override
		{
			m_mousePos  = m_camera->screenToWorld(Vec2(event.mouseMove.x, event.mouseMove.y));
		}
	private:
		Vec2 m_currentPos = Vec2(1, 1);
		Vec2 m_mousePos = Vec2(1, 1);
		Rectangle rectangle;
		Circle circle;
		Polygon polygon;
		Capsule capsule;
		Polygon triangle;
	};
}
#endif