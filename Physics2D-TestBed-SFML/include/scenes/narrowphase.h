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
			polygon1.append({ {0.0f, 4.0f},{-3.0f, 3.0f},{-4.0f, 0.0f},{-3.0f, -3.0f},{0, -4.0f},
			{3.0f, -3.0f}, {4.0f, 0.0f }, {3.0f, 3.0f },{0.0f, 4.0f } });

			polygon2.append({ {0.0f, 4.0f},{-3.0f, 3.0f},{-4.0f, 0.0f},{-3.0f, -3.0f},{0, -4.0f},
			{3.0f, -3.0f}, {4.0f, 0.0f }, {3.0f, 3.0f },{0.0f, 4.0f } });

			circle.setRadius(2.0f);
			ellipse.set(2.0f, 1.0f);

			shape1.shape = &polygon1;
			shape1.transform.position.set(1.0f, 2.0f);
			shape1.transform.rotation = Math::degreeToRadian(30);

			shape2.shape = &polygon2;
			shape2.transform.position.set(1.0f, -5.0f);
			shape2.transform.rotation = Math::degreeToRadian(30);
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
		void render(sf::RenderWindow& window) override
		{
			RenderSFMLImpl::renderShape(window, *m_camera, shape1, sf::Color::Green);
			RenderSFMLImpl::renderShape(window, *m_camera, shape2, sf::Color::Cyan);

			RenderSFMLImpl::renderPoint(window, *m_camera, shape1.transform.position, sf::Color::Green);
			RenderSFMLImpl::renderPoint(window, *m_camera, shape2.transform.position, sf::Color::Cyan);
			Simplex simplex = Narrowphase::gjk(shape1, shape2);
			sf::Color color = simplex.isContainOrigin ? sf::Color::Red : sf::Color::Magenta;
			RenderSFMLImpl::renderSimplex(window, *m_camera, simplex, color);
			if(simplex.isContainOrigin)
			{
				//draw polytope
				auto [finalSimplex, finalList] = Narrowphase::epa(simplex, shape1, shape2);
				RenderSFMLImpl::renderSimplex(window, *m_camera, finalSimplex, sf::Color::Yellow);
			}
			if(isPicked)
			{
				RenderSFMLImpl::renderArrow(window, *m_camera, mousePos, currentPos, sf::Color::Yellow);
			}
			// draw cull

			Container::Vector<Vector2> vertices;
			vertices.reserve(polygon1.vertices().size() * polygon2.vertices().size());
			for (const Vector2& v1 : polygon1.vertices())
			{
				Vector2 p1 = shape1.transform.translatePoint(v1);
				for (const Vector2& v2 : polygon2.vertices())
				{
					Vector2 p2 = shape2.transform.translatePoint(v2);
					vertices.emplace_back(p1 - p2);
				}
			}
			Container::Vector<Vector2> newVertices = GeometryAlgorithm2D::grahamScan(vertices);
			for (auto&& vertex : newVertices)
			{
				RenderSFMLImpl::renderPoint(window, *m_camera, vertex, sf::Color::Cyan);
			}
		}
	private:
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