#ifndef PHYSICS2D_SCENES_NARROWPHASE_H
#define PHYSICS2D_SCENES_NARROWPHASE_H
#include "frame.h"

namespace Physics2D
{
	class NarrowphaseFrame : public Frame
	{
	public:
		NarrowphaseFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Camera* camera) : Frame("Narrow Phase", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{
			brick.set(1.5f, 0.5f);

			block.set(10, 0.1f);
			edge.set(Vector2{ -10.0f, 0.0f }, Vector2{ 10.0f, 0.0f });

			rectangle.set(2, 3);
			polygon1.append({ {0.0f, 4.0f},{-3.0f, 3.0f},{-4.0f, 0.0f},{-3.0f, -3.0f},{0, -4.0f},
			{3.0f, -3.0f}, {4.0f, 0.0f }, {3.0f, 3.0f } });

			polygon2.append({ {0.0f, 4.0f},{-3.0f, 3.0f},{-4.0f, 0.0f},{-3.0f, -3.0f},{0, -4.0f},
			{3.0f, -3.0f}, {4.0f, 0.0f }, {3.0f, 3.0f }});

			circle.setRadius(2.0f);
			ellipse.set(4.0f, 2.0f);

			shape1.shape = &circle;
			shape1.transform.position.set(0.0f, 2.0f);
			//shape1.transform.rotation = Math::degreeToRadian(50);

			shape2.shape = &ellipse;
			shape2.transform.position.set(0.0f, -5.0f);
			//shape2.transform.rotation = Math::degreeToRadian(30);
			//result = Detector::detect(shape1, shape2);

		}
		void onMousePress(sf::Event& event) override
		{
			if (event.mouseButton.button == sf::Mouse::Left)
			{
				mousePos = m_camera->screenToWorld(Vector2(event.mouseButton.x, event.mouseButton.y));
				if (shape1.contains(mousePos))
				{
					isPicked = true;
					clickObject = &shape1;
					originTransform = shape1.transform.position;
					return;
				}
				if (shape2.contains(mousePos))
				{
					isPicked = true;
					clickObject = &shape2;
					originTransform = shape2.transform.position;
				}
			}
		}
		void onMouseMove(sf::Event& event) override
		{
			if (!isPicked)
				return;
			Vector2 pos(real(event.mouseMove.x), real(event.mouseMove.y));
			currentPos = m_camera->screenToWorld(pos);
			Vector2 tf = currentPos - mousePos;

			clickObject->transform.position = originTransform + tf;

		}
		void onMouseRelease(sf::Event& event) override
		{
			isPicked = false;
			originTransform.clear();
			clickObject = nullptr;
		}
		void onKeyPressed(sf::Event& event) override
		{
			if(event.key.code == sf::Keyboard::R)
			{
				shape1.transform.rotation += Math::degreeToRadian(1);
			}
		}
		void render(sf::RenderWindow& window) override
		{
			RenderSFMLImpl::renderShape(window, *m_camera, shape1, sf::Color::Green);
			RenderSFMLImpl::renderShape(window, *m_camera, shape2, sf::Color::Cyan);

			RenderSFMLImpl::renderPoint(window, *m_camera, shape1.transform.position, sf::Color::Green);
			RenderSFMLImpl::renderPoint(window, *m_camera, shape2.transform.position, sf::Color::Cyan);
			Simplex simplex = Narrowphase::gjk(shape1, shape2);
			sf::Color color = simplex.isContainOrigin ? sf::Color::Magenta : sf::Color::Blue;
			//RenderSFMLImpl::renderSimplex(window, *m_camera, simplex, color);
			if(simplex.isContainOrigin)
			{
				//draw polytope
				auto info = Narrowphase::epa(simplex, shape1, shape2);

				for(auto iter = info.polytope.begin(); iter != info.polytope.end(); ++iter)
				{
					auto next = iter;
					++next;
					if(next == info.polytope.end())
						next = info.polytope.begin();

					RenderSFMLImpl::renderLine(window, *m_camera, iter->vertex.result, next->vertex.result, sf::Color::Red);
					RenderSFMLImpl::renderPoint(window, *m_camera, iter->vertex.result, sf::Color::Red);
					RenderSFMLImpl::renderPoint(window, *m_camera, next->vertex.result, sf::Color::Red);
				}
				
				//RenderSFMLImpl::renderSimplex(window, *m_camera, info.simplex, sf::Color::Green);
				RenderSFMLImpl::renderArrow(window, *m_camera, shape1.transform.position, shape1.transform.position + info.normal * info.penetration, sf::Color::Green);

				//RenderSFMLImpl::renderLine(window, *m_camera, info.simplex.vertices[0].point[0], info.simplex.vertices[1].point[0], sf::Color::Yellow);
				//RenderSFMLImpl::renderPoint(window, *m_camera, info.simplex.vertices[0].point[0], sf::Color::Yellow, 4);
				//RenderSFMLImpl::renderPoint(window, *m_camera, info.simplex.vertices[1].point[0], sf::Color::Yellow);

				//RenderSFMLImpl::renderLine(window, *m_camera, info.simplex.vertices[0].point[1], info.simplex.vertices[1].point[1], sf::Color::Magenta);
				//RenderSFMLImpl::renderPoint(window, *m_camera, info.simplex.vertices[0].point[1], sf::Color::Magenta, 4);
				//RenderSFMLImpl::renderPoint(window, *m_camera, info.simplex.vertices[1].point[1], sf::Color::Magenta);
				auto pairs = Narrowphase::generateContacts(shape1, shape2, info);
				sf::Color color1 = sf::Color(239, 103, 50);
				sf::Color color2 = sf::Color(252, 236, 86);
				if(pairs.count == 2)
				{
					RenderSFMLImpl::renderLine(window, *m_camera, pairs.points[0], pairs.points[1], sf::Color::Magenta);
					RenderSFMLImpl::renderPoint(window, *m_camera, pairs.points[0], color1);
					RenderSFMLImpl::renderPoint(window, *m_camera, pairs.points[1], color2);
				}
				else if(pairs.count == 4)
				{
					RenderSFMLImpl::renderLine(window, *m_camera, pairs.points[0], pairs.points[1], sf::Color::Magenta);
					RenderSFMLImpl::renderLine(window, *m_camera, pairs.points[2], pairs.points[3], sf::Color::Magenta);
					RenderSFMLImpl::renderPoint(window, *m_camera, pairs.points[0], color1);
					RenderSFMLImpl::renderPoint(window, *m_camera, pairs.points[1], color2);
					RenderSFMLImpl::renderPoint(window, *m_camera, pairs.points[2], color1);
					RenderSFMLImpl::renderPoint(window, *m_camera, pairs.points[3], color2);
				}
			}
			if(isPicked)
			{
				RenderSFMLImpl::renderArrow(window, *m_camera, mousePos, currentPos, sf::Color::Yellow);
			}
			// draw cull
		}
	private:

		Edge edge;
		Rectangle block;

		Rectangle brick;

		Rectangle rectangle;
		Polygon polygon1;
		Polygon polygon2;
		Circle circle;
		Ellipse ellipse;
		ShapePrimitive shape1, shape2;
		//Collision result;
		bool isPicked = false;
		Vector2 mousePos;
		Vector2 currentPos;
		Vector2 originTransform;
		ShapePrimitive* clickObject = nullptr;
	};
}
#endif