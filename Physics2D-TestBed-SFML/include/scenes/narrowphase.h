#ifndef PHYSICS2D_SCENES_NARROWPHASE_H
#define PHYSICS2D_SCENES_NARROWPHASE_H
#include "frame.h"

namespace Physics2D
{
	class NarrowphaseFrame : public Frame
	{
	public:
		NarrowphaseFrame(const FrameSettings& settings) : Frame(settings)
		{
		}

		void onLoad() override
		{
			capsule.set(4.0f, 2.0f);
			capsule2.set(2.0f, 0.5f);
			triangle.append({{-1.0f, 1.0f}, {0.0f, -2.0f}, {1.0f, -1.0f}});
			smallBrick.set(1.0f, 1.0f);

			brick.set(1.5f, 0.5f);

			block.set(10, 0.1f);
			edge.set(Vector2{-100.0f, 0.0f}, Vector2{100.0f, 0.0f});

			rectangle.set(0.3f, 3.0f);
			polygon1.append({
				{0.0f, 4.0f}, {-3.0f, 3.0f}, {-4.0f, 0.0f}, {-3.0f, -3.0f}, {0, -4.0f},
				{3.0f, -3.0f}, {4.0f, 0.0f}, {3.0f, 3.0f}
			});

			polygon1.scale(0.5f);

			polygon2.append({{-1.0f, 1.0f}, {0.0f, -2.0f}, {1.0f, -1.0f}});

			circle.setRadius(1.0f);
			ellipse.set(2.0f * std::sqrt(2.0f), 2.0f);

			wall.set(60.0, 200.0f);

			//shape1.shape = &smallBrick;
			//shape1.transform.position.set(-0.500000477f, 0.499320120f);
			//shape1.transform.rotation = 9.10090932e-07f;

			//shape2.shape = &smallBrick;
			//shape2.transform.position.set(0.500000477f, 0.499320120f);
			//shape2.transform.rotation = -9.10090932e-07f;

			shape1.shape = &polygon1;
			shape1.transform.position.set(0.0f, 2.0f);
			//shape1.transform.position.set(-0.2875f, -1.6625f);
			//shape1.transform.rotation = 9.10090932e-07f;

			shape2.shape = &triangle;
			shape2.transform.position.set(-2.0f, -2.0f);
			//shape2.transform.rotation = Math::degreeToRadian(45);

			//result = Detector::detect(shape1, shape2);
		}

		void onMousePress(sf::Event& event) override
		{
			if (event.mouseButton.button == sf::Mouse::Left)
			{
				mousePos = m_settings.camera->screenToWorld(Vector2(event.mouseButton.x, event.mouseButton.y));
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
			Vector2 pos(static_cast<real>(event.mouseMove.x), static_cast<real>(event.mouseMove.y));
			currentPos = m_settings.camera->screenToWorld(pos);
			Vector2 tf = currentPos - mousePos;

			clickObject->transform.position = originTransform + tf;
		}

		void onMouseRelease(sf::Event& event) override
		{
			isPicked = false;
			mousePos.clear();
			currentPos.clear();
			originTransform.clear();
			clickObject = nullptr;
		}

		void onKeyPressed(sf::Event& event) override
		{
			if (event.key.code == sf::Keyboard::E)
			{
				shape1.transform.rotation += Math::degreeToRadian(1.0f);
			}
		}

		void onPostRender(sf::RenderWindow& window) override
		{
			RenderSFMLImpl::renderShape(window, *m_settings.camera, shape1, sf::Color::Green);
			RenderSFMLImpl::renderShape(window, *m_settings.camera, shape2, sf::Color::Cyan);

			RenderSFMLImpl::renderPoint(window, *m_settings.camera, shape1.transform.position, sf::Color::Green);
			RenderSFMLImpl::renderPoint(window, *m_settings.camera, shape2.transform.position, sf::Color::Cyan);
			auto info = Narrowphase::gjkDistance(shape1, shape2);

			if(showPolytope)
			{
				std::vector<Vector2> polytope;
				for (auto&& elem : info.polytope)
					polytope.emplace_back(elem.vertex.result);
				RenderSFMLImpl::renderPolytope(window, *m_settings.camera, polytope, RenderConstant::Pink, *m_settings.font);
			}

			if (showOriginalSimplex)
				RenderSFMLImpl::renderSimplex(window, *m_settings.camera, info.originalSimplex, RenderConstant::Yellow, *m_settings.font);

			if (showFeatureSimplex)
				RenderSFMLImpl::renderSimplex(window, *m_settings.camera, info.simplex, RenderConstant::Yellow, *m_settings.font, false);

			//Simplex simplex = Narrowphase::gjk(shape1, shape2);
			//sf::Color color = simplex.isContainOrigin ? RenderConstant::Teal : RenderConstant::Orange;

			//if(showGJKSimplex)
			//	RenderSFMLImpl::renderSimplex(window, *m_settings.camera, simplex, color);
			//if(simplex.isContainOrigin)
			//{
			//	//draw polytope
			//	auto info = Narrowphase::epa(simplex, shape1, shape2);

			//	for(auto iter = info.polytope.begin(); iter != info.polytope.end(); ++iter)
			//	{
			//		auto next = iter;
			//		++next;
			//		if(next == info.polytope.end())
			//			next = info.polytope.begin();

			//			if (showPolytope)
			//			{
			//				RenderSFMLImpl::renderLine(window, *m_settings.camera, iter->vertex.result, next->vertex.result,
			//				                           RenderConstant::Pink);
			//				RenderSFMLImpl::renderPoint(window, *m_settings.camera, iter->vertex.result, RenderConstant::Pink, 3);
			//				RenderSFMLImpl::renderPoint(window, *m_settings.camera, next->vertex.result, RenderConstant::Pink, 3);

			//				RenderSFMLImpl::renderInt(window, *m_settings.camera, iter->vertex.result, m_settings.camera->font(),
			//				                            std::distance(info.polytope.begin(), iter),
			//				                            RenderConstant::Pink, 14, {-0.25f, -0.25f});
			//			
			//			}
			//	}
			//////	
			//////	//RenderSFMLImpl::renderSimplex(window, *m_settings.camera, info.simplex, RenderConstant::Teal);
			//////	//Vector2 p = GeometryAlgorithm2D::pointToLineSegment(info.simplex.vertices[0].result,
			//////	//	info.simplex.vertices[1].result, { 0, 0 });
			//////	//RenderSFMLImpl::renderPoint(window, *m_settings.camera, p, RenderConstant::Orange);
			//////	//RenderSFMLImpl::renderLine(window, *m_settings.camera, p, { 0,0 }, RenderConstant::Orange);

			auto color1 = sf::Color(239, 103, 50);
			auto color2 = sf::Color(252, 236, 86);

			////////	//draw feature simplex
			if (showFeature)
			{
				RenderSFMLImpl::renderPoint(window, *m_settings.camera, info.simplex.vertices[0].point[0], color1);
				RenderSFMLImpl::renderPoint(window, *m_settings.camera, info.simplex.vertices[1].point[0], color1);
				RenderSFMLImpl::renderLine(window, *m_settings.camera, info.simplex.vertices[0].point[0],
				                           info.simplex.vertices[1].point[0], color1);

				RenderSFMLImpl::renderPoint(window, *m_settings.camera, info.simplex.vertices[0].point[1], color2);
				RenderSFMLImpl::renderPoint(window, *m_settings.camera, info.simplex.vertices[1].point[1], color2);
				RenderSFMLImpl::renderLine(window, *m_settings.camera, info.simplex.vertices[0].point[1],
				                           info.simplex.vertices[1].point[1], color2);

				RenderSFMLImpl::renderPoint(window, *m_settings.camera, info.pair.pointA, color1);
				RenderSFMLImpl::renderPoint(window, *m_settings.camera, info.pair.pointB, color2);
				RenderSFMLImpl::renderLine(window, *m_settings.camera, info.pair.pointA,info.pair.pointB, RenderConstant::Gray);

				RenderSFMLImpl::renderPosition(window, *m_settings.camera, info.pair.pointA, color1, *m_settings.font);
				RenderSFMLImpl::renderPosition(window, *m_settings.camera, info.pair.pointB, color2, *m_settings.font);
				
			}
			//////	

			//////	
			//////	
			//	//RenderSFMLImpl::renderLine(window, *m_settings.camera, info.simplex.vertices[0].point[0], info.simplex.vertices[1].point[0], sf::Color::Yellow);
			//	//RenderSFMLImpl::renderPoint(window, *m_settings.camera, info.simplex.vertices[0].point[0], sf::Color::Yellow, 4);
			//	//RenderSFMLImpl::renderPoint(window, *m_settings.camera, info.simplex.vertices[1].point[0], sf::Color::Yellow);

			//	//RenderSFMLImpl::renderLine(window, *m_settings.camera, info.simplex.vertices[0].point[1], info.simplex.vertices[1].point[1], sf::Color::Magenta);
			//	//RenderSFMLImpl::renderPoint(window, *m_settings.camera, info.simplex.vertices[0].point[1], sf::Color::Magenta, 4);
			//	//RenderSFMLImpl::renderPoint(window, *m_settings.camera, info.simplex.vertices[1].point[1], sf::Color::Magenta);
			//	auto pairs = Narrowphase::generateContacts(shape1, shape2, info);
			//	if(pairs.count == 2)
			//	{
			//		RenderSFMLImpl::renderLine(window, *m_settings.camera, pairs.points[0], pairs.points[1], sf::Color::Magenta);
			//		RenderSFMLImpl::renderPoint(window, *m_settings.camera, pairs.points[0], color1);
			//		RenderSFMLImpl::renderPoint(window, *m_settings.camera, pairs.points[1], color2);
			//	}
			//	else if(pairs.count == 4)
			//	{
			//		RenderSFMLImpl::renderLine(window, *m_settings.camera, pairs.points[0], pairs.points[1], sf::Color::Magenta);
			//		RenderSFMLImpl::renderLine(window, *m_settings.camera, pairs.points[2], pairs.points[3], sf::Color::Magenta);
			//		RenderSFMLImpl::renderPoint(window, *m_settings.camera, pairs.points[0], color1);
			//		RenderSFMLImpl::renderPoint(window, *m_settings.camera, pairs.points[1], color2);
			//		RenderSFMLImpl::renderPoint(window, *m_settings.camera, pairs.points[2], color1);
			//		RenderSFMLImpl::renderPoint(window, *m_settings.camera, pairs.points[3], color2);
			//	}
			////////	////draw final normal

			//////	RenderSFMLImpl::renderArrow(window, *m_settings.camera, shape1.transform.position, shape1.transform.position + info.normal * info.penetration, sf::Color::Green);
			//////	RenderSFMLImpl::renderArrow(window, *m_settings.camera, shape1.transform.position, shape1.transform.position + info.normal, sf::Color::Cyan);

			//}
			// draw hull
			if (showHull)
			{
				//for (auto& a : polygon1.vertices())
				//{
				//	for (auto& b : triangle.vertices())
				//	{
				//		Vector2 p1 = shape1.transform.translatePoint(a);
				//		Vector2 p2 = shape2.transform.translatePoint(b);
				//		Vector2 v = p1 - p2;
				//		RenderSFMLImpl::renderPoint(window, *m_settings.camera, v, RenderConstant::Yellow);
				//	}
				//}
			}

			if (isPicked && showMouseTransform)
			{
				if (mousePos.isOrigin() || currentPos.isOrigin())
					return;

				RenderSFMLImpl::renderArrow(window, *m_settings.camera, mousePos, currentPos, sf::Color::Yellow);
			}
		}

		void onRenderUI() override
		{
			Vector2 pos(10.0f, 0.0f);
			pos = m_settings.camera->worldToScreen(pos);
			ImGui::SetNextWindowPos(ImVec2(pos.x, pos.y), ImGuiCond_Once);

			ImGui::Begin("Distance");
			ImGui::Checkbox("Show Polytope", &showPolytope);
			//ImGui::Checkbox("Show GJK Simplex", &showGJKSimplex);
			ImGui::Checkbox("Show Original Simplex", &showOriginalSimplex);
			ImGui::Checkbox("Show Closest Feature", &showFeature);
			ImGui::Checkbox("Show Feature Simplex", &showFeatureSimplex);
			//ImGui::Checkbox("Show Hull", &showHull);
			ImGui::Checkbox("Show Mouse Transform", &showMouseTransform);
			ImGui::Checkbox("Show Hull", &showHull);

			ImGui::End();
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

		bool showPolytope = true;
		bool showFeature = true;
		bool showFeatureSimplex = true;
		bool showOriginalSimplex = false;
		bool showMouseTransform = false;

		bool showHull = false;

		bool showGJKSimplex = false;
		Vector2 mousePos;
		Vector2 currentPos;
		Vector2 originTransform;
		ShapePrimitive* clickObject = nullptr;
	};
}
#endif
