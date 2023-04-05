#ifndef PHYSICS2D_SCENES_NARROWPHASE_H
#define PHYSICS2D_SCENES_NARROWPHASE_H
#include "frame.h"

namespace Physics2D
{
	class NarrowphaseFrame : public Frame
	{
	public:
		NarrowphaseFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, UniformGrid* grid, Camera* camera) : Frame("Narrowphase", world, maintainer, tree, grid, camera)
		{

		}
		void load() override
		{
			capsule.set(4.0f, 2.0f);
			capsule2.set(2.0f, 0.5f);
			triangle.append({ {-1.0f, 1.0f},{0.0f, -2.0f},{1.0f, -1.0f} });
			smallBrick.set(1.0f, 1.0f);

			brick.set(1.5f, 0.5f);

			block.set(10, 0.1f);
			edge.set(Vector2{ -100.0f, 0.0f }, Vector2{ 100.0f, 0.0f });

			rectangle.set(0.3, 3.0);
			polygon1.append({ {0.0f, 4.0f},{-3.0f, 3.0f},{-4.0f, 0.0f},{-3.0f, -3.0f},{0, -4.0f},
			{3.0f, -3.0f}, {4.0f, 0.0f }, {3.0f, 3.0f } });

			polygon2.append({ {-1.0f, 1.0f},{0.0f, -2.0f},{1.0f, -1.0f} });

			circle.setRadius(2.0f);
			ellipse.set(4.0f, 2.0f);

			wall.set(60.0, 200.0f);

			//shape1.shape = &smallBrick;
			//shape1.transform.position.set(-0.500000477f, 0.499320120f);
			//shape1.transform.rotation = 9.10090932e-07f;

			//shape2.shape = &smallBrick;
			//shape2.transform.position.set(0.500000477f, 0.499320120f);
			//shape2.transform.rotation = -9.10090932e-07f;

			shape1.shape = &capsule;
			shape1.transform.position.set(0.0, 3.0f);
			//shape1.transform.rotation = 9.10090932e-07f;

			shape2.shape = &block;
			shape2.transform.position.set(0.0f, 0.0f);
			//shape2.transform.rotation = -9.10090932e-07f;

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
			if(event.key.code == sf::Keyboard::E)
			{
				shape1.transform.rotation += Math::degreeToRadian(1.0f);
			}
		}
		void render(sf::RenderWindow& window) override
		{

			RenderSFMLImpl::renderShape(window, *m_camera, shape1, sf::Color::Green);
			RenderSFMLImpl::renderShape(window, *m_camera, shape2, sf::Color::Cyan);

			RenderSFMLImpl::renderPoint(window, *m_camera, shape1.transform.position, sf::Color::Green);
			RenderSFMLImpl::renderPoint(window, *m_camera, shape2.transform.position, sf::Color::Cyan);
			//auto info = Narrowphase::gjkDistance(shape1, shape2);
			//for (auto iter = info.polytope.begin(); iter != info.polytope.end(); ++iter)
			//{
			//	auto next = iter;
			//	++next;
			//	if (next == info.polytope.end())
			//		next = info.polytope.begin();

			//	RenderSFMLImpl::renderLine(window, *m_camera, iter->vertex.result, next->vertex.result, RenderConstant::MaterialPink);
			//	RenderSFMLImpl::renderPoint(window, *m_camera, iter->vertex.result, RenderConstant::MaterialPink);
			//	RenderSFMLImpl::renderPoint(window, *m_camera, next->vertex.result, RenderConstant::MaterialPink);
			//}
			Simplex simplex = Narrowphase::gjk(shape1, shape2);
			sf::Color color = simplex.isContainOrigin ? RenderConstant::MaterialTeal : RenderConstant::MaterialOrange;


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

					RenderSFMLImpl::renderLine(window, *m_camera, iter->vertex.result, next->vertex.result, RenderConstant::MaterialPink);
					RenderSFMLImpl::renderPoint(window, *m_camera, iter->vertex.result, RenderConstant::MaterialPink);
					RenderSFMLImpl::renderPoint(window, *m_camera, next->vertex.result, RenderConstant::MaterialPink);
				}
			//	
			//	//RenderSFMLImpl::renderSimplex(window, *m_camera, info.simplex, RenderConstant::MaterialTeal);
			//	//Vector2 p = GeometryAlgorithm2D::pointToLineSegment(info.simplex.vertices[0].result,
			//	//	info.simplex.vertices[1].result, { 0, 0 });
			//	//RenderSFMLImpl::renderPoint(window, *m_camera, p, RenderConstant::MaterialOrange);
			//	//RenderSFMLImpl::renderLine(window, *m_camera, p, { 0,0 }, RenderConstant::MaterialOrange);

				sf::Color color1 = sf::Color(239, 103, 50);
				sf::Color color2 = sf::Color(252, 236, 86);

			//	//draw feature simplex

				//RenderSFMLImpl::renderPoint(window, *m_camera, info.simplex.vertices[0].point[0], color1);
				//RenderSFMLImpl::renderPoint(window, *m_camera, info.simplex.vertices[1].point[0], color1);
				//RenderSFMLImpl::renderLine(window, *m_camera, info.simplex.vertices[0].point[0], 
				//	info.simplex.vertices[1].point[0], color1);

				//RenderSFMLImpl::renderPoint(window, *m_camera, info.simplex.vertices[0].point[1], color2);
				//RenderSFMLImpl::renderPoint(window, *m_camera, info.simplex.vertices[1].point[1], color2);
				//RenderSFMLImpl::renderLine(window, *m_camera, info.simplex.vertices[0].point[1],
				//	info.simplex.vertices[1].point[1], color2);
			//	

			//	
			//	
				//RenderSFMLImpl::renderLine(window, *m_camera, info.simplex.vertices[0].point[0], info.simplex.vertices[1].point[0], sf::Color::Yellow);
				//RenderSFMLImpl::renderPoint(window, *m_camera, info.simplex.vertices[0].point[0], sf::Color::Yellow, 4);
				//RenderSFMLImpl::renderPoint(window, *m_camera, info.simplex.vertices[1].point[0], sf::Color::Yellow);

				//RenderSFMLImpl::renderLine(window, *m_camera, info.simplex.vertices[0].point[1], info.simplex.vertices[1].point[1], sf::Color::Magenta);
				//RenderSFMLImpl::renderPoint(window, *m_camera, info.simplex.vertices[0].point[1], sf::Color::Magenta, 4);
				//RenderSFMLImpl::renderPoint(window, *m_camera, info.simplex.vertices[1].point[1], sf::Color::Magenta);
				auto pairs = Narrowphase::generateContacts(shape1, shape2, info);
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
			//	////draw final normal

				RenderSFMLImpl::renderArrow(window, *m_camera, shape1.transform.position, shape1.transform.position + info.normal * info.penetration, sf::Color::Green);
				RenderSFMLImpl::renderArrow(window, *m_camera, shape1.transform.position, shape1.transform.position + info.normal, sf::Color::Cyan);

			}
			if(isPicked)
			{
				RenderSFMLImpl::renderArrow(window, *m_camera, mousePos, currentPos, sf::Color::Yellow);
			}
			// draw cull
		}
	private:

		Capsule capsule;
		Capsule capsule2;
		Edge edge;
		Rectangle block;

		Rectangle brick;
		Rectangle wall;

		Polygon triangle;

		Rectangle smallBrick;
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