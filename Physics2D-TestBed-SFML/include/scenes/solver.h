#ifndef PHYSICS2D_SCENES_SOLVER_H
#define PHYSICS2D_SCENES_SOLVER_H
#include "frame.h"

namespace Physics2D
{
	class SolverFrame : public Frame
	{
	public:
		SolverFrame(const FrameSettings& settings) : Frame(settings)
		{
		}

		void onLoad() override
		{
			float min = std::min(m_halfWidth, m_halfHeight);
			float maxRadius = min * m_percentage;
			float radius = maxRadius * m_percentage;
			
			m_bezierPoints1[1] = Vector2(m_halfWidth - radius, m_halfHeight);
			m_bezierPoints1[2] = Vector2(m_halfWidth - radius * 0.6f, m_halfHeight);
		}

		void onPostRender(sf::RenderWindow& window) override
		{
			std::vector<Vector2> vertices;

			std::vector<Vector2> newVertices;

			Vector2 p0(m_halfWidth, 0);
			Vector2 p1(0, m_halfHeight);

			vertices.push_back(p0);
			newVertices.push_back(p1);

			float min = std::min(m_halfWidth, m_halfHeight);
			float maxRadius = min * m_percentage;
			float radius = maxRadius * m_percentage;

			float step = Constant::Pi * 0.5f / m_count;

			Vector2 center(m_halfWidth - radius, m_halfHeight - radius);

			Vector2 corner(radius * std::cos(Math::degreeToRadian(45)), radius * std::sin(Math::degreeToRadian(45)));
			corner += center;

			m_cornerCenter = center;

			for (float i = 0.0f; i <= m_count; i += 1.0f)
			{
				float angle = i * step;
				Vector2 p(radius * std::cos(angle), radius * std::sin(angle));
				p += center;
				vertices.push_back(p);
			}

			vertices.push_back(p1);


			float innerRadius = min - radius;
			innerRadius *= m_innerRadiusPercentage;

			std::vector<Vector2> innerCircle;

			float innerCount = 4.0f * m_count;
			float innerStep = 2.0f * Constant::Pi / innerCount;
			for (float i = 0.0f; i <= innerCount; i += 1.0f)
			{
				float angle = i * innerStep;
				Vector2 p(innerRadius * std::cos(angle), innerRadius * std::sin(angle));
				innerCircle.push_back(p);
				if (i == 0.0f)
					continue;

				if(m_showReferenceLine)
					RenderSFMLImpl::renderLine(window, *m_settings.camera, innerCircle[i - 1], innerCircle[i], gray);
			}

			//reference line

			if (m_showReferenceLine)
				RenderSFMLImpl::renderLine(window, *m_settings.camera, center, corner, gray);

			Vector2 refP1(innerRadius, m_halfHeight);
			Vector2 refP2(innerRadius, 0.0f);
			Vector2 refP3(m_halfWidth, innerRadius);
			Vector2 refP4(0.0f, innerRadius);

			if (m_showReferenceLine)
			{
				RenderSFMLImpl::renderLine(window, *m_settings.camera, refP1, refP2, gray);
				RenderSFMLImpl::renderLine(window, *m_settings.camera, refP3, refP4, gray);
			}

			refP1 = Vector2(m_halfWidth, m_halfHeight - radius);
			refP2 = Vector2(0.0f, m_halfHeight - radius);
			refP3 = Vector2(m_halfWidth - radius, m_halfHeight);
			refP4 = Vector2(m_halfWidth - radius, 0.0f);
			if (m_showReferenceLine)
			{
				RenderSFMLImpl::renderLine(window, *m_settings.camera, refP1, refP2, gray);
				RenderSFMLImpl::renderLine(window, *m_settings.camera, refP3, refP4, gray);
			}
			refP1 = Vector2(corner.x, 0.0f);
			refP2 = Vector2(0.0f, corner.y);
			if (m_showReferenceLine)
			{
				RenderSFMLImpl::renderLine(window, *m_settings.camera, corner, refP1, gray);
				RenderSFMLImpl::renderLine(window, *m_settings.camera, corner, refP2, gray);
			}
		
			float roundedAngle = Math::degreeToRadian(45.0f * m_cornerPercentage);
			float startRounded = Math::degreeToRadian(45.0f) - roundedAngle;
			float endRounded = Math::degreeToRadian(45.0f) + roundedAngle;

			refP1 = radius * Vector2(std::cos(startRounded), std::sin(startRounded));
			refP2 = radius * Vector2(std::cos(endRounded), std::sin(endRounded));
			refP1 += center;
			refP2 += center;

			if (m_showReferenceLine)
			{
				RenderSFMLImpl::renderLine(window, *m_settings.camera, center, refP1, gray);
				RenderSFMLImpl::renderLine(window, *m_settings.camera, center, refP2, gray);
			}

			m_endRoundedPos = refP2;
			m_startRoundedPos = refP1;


			m_bezierPoints1[0] = Vector2(innerRadius, m_halfHeight);
			m_bezierPoints1[3] = refP2;

			m_bezierPoints2[0] = Vector2(m_halfWidth, innerRadius);
			m_bezierPoints2[3] = refP1;

			//RenderSFMLImpl::renderPoint(window, *m_settings.camera, m_bezierPoints[0], color, 4.0f);
			//RenderSFMLImpl::renderPoint(window, *m_settings.camera, m_bezierPoints[3], color, 4.0f);
			//RenderSFMLImpl::renderPoint(window, *m_settings.camera, m_bezierPoints[1], gray, 4.0f);
			//RenderSFMLImpl::renderPoint(window, *m_settings.camera, m_bezierPoints[2], gray, 4.0f);

			std::vector<Vector2> bezierSamplePoints1, bezierSamplePoints2;
			bezierSamplePoints1.reserve(m_bezierCount);
			float bezierStep = 1.0f / m_bezierCount;
			for (float t = 0.0f; t < 1.0f; t += bezierStep) {
				Vector2 p = std::pow(1.0f - t, 3.0f) * m_bezierPoints1[0] +
					3.0f * t * std::pow(1.0f - t, 2.0f) * m_bezierPoints1[1] +
					3.0f * t * t * (1 - t) * m_bezierPoints1[2] +
					std::pow(t, 3.0f) * m_bezierPoints1[3];

				bezierSamplePoints1.push_back(p);
				newVertices.push_back(p);

				p = std::pow(1.0f - t, 3.0f) * m_bezierPoints2[0] +
					3.0f * t * std::pow(1.0f - t, 2.0f) * m_bezierPoints2[1] +
					3.0f * t * t * (1 - t) * m_bezierPoints2[2] +
					std::pow(t, 3.0f) * m_bezierPoints2[3];

				bezierSamplePoints2.push_back(p);

				if (t == 0.0f)
					continue;

				//RenderSFMLImpl::renderLine(window, *m_settings.camera,
				//	bezierSamplePoints1[bezierSamplePoints1.size() - 2], bezierSamplePoints1[bezierSamplePoints1.size() - 1], RenderConstant::Green);

				//RenderSFMLImpl::renderLine(window, *m_settings.camera,
				//	bezierSamplePoints2[bezierSamplePoints2.size() - 2], bezierSamplePoints2[bezierSamplePoints2.size() - 1], RenderConstant::Green);


			}


			float k = 1.0f / radius;
			float scaleK = 1.0f * k;
			//draw curvature of rounded
			std::vector<Vector2> curvatureOfRounded;
			std::vector<Vector2> curvatureOfRoundedStart;
			float roundedStep = endRounded - startRounded;
			float roundedCount = m_count * 0.5f;
			roundedStep /= roundedCount;
			for (float i = 0; i <= roundedCount; i += 1.0f)
			{
				float angle = i * roundedStep + startRounded;
				Vector2 from(std::cos(angle), std::sin(angle));
				from *= radius;
				from += center;
				Vector2 point(std::cos(angle), std::sin(angle));
				point *= scaleK * m_curvatureScaleFactor;
				point += from;

				curvatureOfRounded.push_back(point);
				curvatureOfRoundedStart.push_back(from);

				if(m_showRoundedCurvature)
					RenderSFMLImpl::renderLine(window, *m_settings.camera, from, point, gray);

				if (i == 0)
					continue;

				if (m_showRoundedCurvature)
					RenderSFMLImpl::renderLine(window, *m_settings.camera,
					curvatureOfRounded[curvatureOfRounded.size() - 2], curvatureOfRounded[curvatureOfRounded.size() - 1], gray);
			}

			for(auto iter = curvatureOfRoundedStart.rbegin() + 1; iter != curvatureOfRoundedStart.rend(); ++iter)
			{
				//RenderSFMLImpl::renderPoint(window, *m_settings.camera, *iter, RenderConstant::Blue, 2.0f);
				newVertices.push_back(*iter);
			}
			


			std::vector<Vector2> cubicBezierCurvature1, cubicBezierCurvature2;
			std::vector<float> cubicScaleK1, cubicScaleK2;
			//draw curvature
			for (float t = 0.0f; t <= 1.0f; t += bezierStep)
			{

				Vector2 p1 = std::pow(1.0f - t, 3.0f) * m_bezierPoints1[0] +
					3.0f * t * std::pow(1.0f - t, 2.0f) * m_bezierPoints1[1] +
					3.0f * t * t * (1 - t) * m_bezierPoints1[2] +
					std::pow(t, 3.0f) * m_bezierPoints1[3];

				Vector2 dp = -3.0f * (1.0f - t) * (1.0f - t) * m_bezierPoints1[0] +
					(9.0f * t * t - 12.0f * t + 3.0f) * m_bezierPoints1[1] +
					(6.0f * t - 9.0f * t * t) * m_bezierPoints1[2] +
					3.0f * t * t * m_bezierPoints1[3];

				Vector2 ddp = 6.0f * (1.0f - t) * m_bezierPoints1[0] +
					(18.0f * t - 12.0f) * m_bezierPoints1[1] +
					(6.0f - 18.0f * t) * m_bezierPoints1[2] +
					6.0f * t * m_bezierPoints1[3];

				float k = std::abs(dp.x * ddp.y - dp.y * ddp.x) / std::pow(dp.length(), 3.0f);

				
				//if (k > 0.0f)
				//	kRadius = 1.0f / k;

				float kRadius = 1.0f * k;
				cubicScaleK1.emplace_back(kRadius);

				Vector2 tangent = dp.normal();
				Vector2 normal = tangent.perpendicular();

				Vector2 curvaturePoint1 = normal * kRadius * m_curvatureScaleFactor + p1;

				cubicBezierCurvature1.push_back(curvaturePoint1);

				Vector2 p2 = std::pow(1.0f - t, 3.0f) * m_bezierPoints2[0] +
					3.0f * t * std::pow(1.0f - t, 2.0f) * m_bezierPoints2[1] +
					3.0f * t * t * (1 - t) * m_bezierPoints2[2] +
					std::pow(t, 3.0f) * m_bezierPoints2[3];

				dp = -3.0f * (1.0f - t) * (1.0f - t) * m_bezierPoints2[0] +
					(9.0f * t * t - 12.0f * t + 3.0f) * m_bezierPoints2[1] +
					(6.0f * t - 9.0f * t * t) * m_bezierPoints2[2] +
					3.0f * t * t * m_bezierPoints2[3];

				ddp = 6.0f * (1.0f - t) * m_bezierPoints2[0] +
					(18.0f * t - 12.0f) * m_bezierPoints2[1] +
					(6.0f - 18.0f * t) * m_bezierPoints2[2] +
					6.0f * t * m_bezierPoints2[3];

				k = std::abs(dp.x * ddp.y - dp.y * ddp.x) / std::pow(dp.length(), 3.0f);

				kRadius = 1.0f * k;

				cubicScaleK2.emplace_back(kRadius);

				tangent= dp.normal();
				normal = tangent.perpendicular();

				Vector2 curvaturePoint2 = -normal * kRadius * m_curvatureScaleFactor + p2;

				cubicBezierCurvature2.push_back(curvaturePoint2);

				if(m_showBezierCurvature)
				{
					RenderSFMLImpl::renderLine(window, *m_settings.camera, p1, curvaturePoint1, RenderConstant::Red);
					RenderSFMLImpl::renderLine(window, *m_settings.camera, p2, curvaturePoint2, RenderConstant::Red);

					if (t == 0.0f)
						continue;

					RenderSFMLImpl::renderLine(window, *m_settings.camera,
						cubicBezierCurvature1[cubicBezierCurvature1.size() - 2], cubicBezierCurvature1[cubicBezierCurvature1.size() - 1], RenderConstant::Red);

					RenderSFMLImpl::renderLine(window, *m_settings.camera,
						cubicBezierCurvature2[cubicBezierCurvature2.size() - 2], cubicBezierCurvature2[cubicBezierCurvature2.size() - 1], RenderConstant::Red);
				}

			}
			
			Vector2 endDir = m_cornerCenter - m_endRoundedPos;
			endDir = endDir.normal().perpendicular();
			
			float t = (m_halfHeight - m_endRoundedPos.y) / endDir.y;

			Vector2 p = m_endRoundedPos + t * endDir;

			m_bezierPoints1[2] = p; 

			endDir = m_cornerCenter - m_startRoundedPos;
			endDir = endDir.normal().perpendicular();

			t = (m_halfWidth - m_startRoundedPos.x) / endDir.x;

			p = m_startRoundedPos + t * endDir;

			m_bezierPoints2[2] = p;


			float A = 18.0f * ((m_bezierPoints1[3].x - m_bezierPoints1[2].x) * (m_bezierPoints1[0].y - m_bezierPoints1[2].y)
				- (m_bezierPoints1[0].x - m_bezierPoints1[2].x) * (m_bezierPoints1[3].y - m_bezierPoints1[2].y));
			float C = std::pow(9.0f * (m_bezierPoints1[3] - m_bezierPoints1[2]).dot(m_bezierPoints1[3] - m_bezierPoints1[2]), 3.0f);
			C /= radius * radius;
			
			float u = std::sqrt(C) / std::abs(A);
			Vector2 finalP = m_bezierPoints1[0] * u + (1.0f - u) * m_bezierPoints1[2];

			m_bezierPoints1[1] = finalP;

			A = 18.0f * ((m_bezierPoints2[3].x - m_bezierPoints2[2].x) * (m_bezierPoints2[0].y - m_bezierPoints2[2].y)
				- (m_bezierPoints2[0].x - m_bezierPoints2[2].x) * (m_bezierPoints2[3].y - m_bezierPoints2[2].y));
			C = std::pow(9.0f * (m_bezierPoints2[3] - m_bezierPoints2[2]).dot(m_bezierPoints2[3] - m_bezierPoints2[2]), 3.0f);
			C /= radius * radius;

			u = std::sqrt(C) / std::abs(A);

			finalP = m_bezierPoints2[0] * u + (1.0f - u) * m_bezierPoints2[2];

			m_bezierPoints2[1] = finalP;

			//RenderSFMLImpl::renderPoint(window, *m_settings.camera, finalP, color, 4.0f);

			for(auto&& iter = bezierSamplePoints2.rbegin() + 1; iter != bezierSamplePoints2.rend(); ++iter)
			{
				newVertices.push_back(*iter);
			}

			newVertices.push_back(p0);



			std::vector<Vector2> vertices2, vertices3, vertices4;
			std::vector<Vector2> newVertices2, newVertices3, newVertices4;
			for(auto& p : vertices)
			{
				vertices2.emplace_back(-p.x, p.y);
				vertices3.emplace_back(-p.x, -p.y);
				vertices4.emplace_back(p.x, -p.y);
			}

			for(auto& p : newVertices)
			{
				newVertices2.emplace_back(-p.x, p.y);
				newVertices3.emplace_back(-p.x, -p.y);
				newVertices4.emplace_back(p.x, -p.y);
			}

			if (m_showG2)
			{
				//for(size_t i = 0 ; i < newVertices.size(); ++i)
				//{
				//	RenderSFMLImpl::renderUInt(window, *m_settings.camera, newVertices[i], *m_settings.font,
				//		i, RenderConstant::Green, 18, Vector2(0.0f, 0.0f));
				//	//RenderSFMLImpl::renderPoint(window, *m_settings.camera, elem, RenderConstant::Green, 2.0f);
				//}
				for (size_t i = 1; i < newVertices.size(); ++i)
				{
					RenderSFMLImpl::renderLine(window, *m_settings.camera, newVertices[i - 1], newVertices[i], RenderConstant::Green);
					RenderSFMLImpl::renderLine(window, *m_settings.camera, newVertices2[i - 1], newVertices2[i], RenderConstant::Green);
					RenderSFMLImpl::renderLine(window, *m_settings.camera, newVertices3[i - 1], newVertices3[i], RenderConstant::Green);
					RenderSFMLImpl::renderLine(window, *m_settings.camera, newVertices4[i - 1], newVertices4[i], RenderConstant::Green);
				}
			}

			if (m_showG1)
			{
				for (size_t i = 0; i < vertices.size() - 1; ++i)
				{
					RenderSFMLImpl::renderLine(window, *m_settings.camera, vertices[i], vertices[(i + 1) % vertices.size()], color);
					RenderSFMLImpl::renderLine(window, *m_settings.camera, vertices2[i], vertices2[(i + 1) % vertices2.size()], color);
					RenderSFMLImpl::renderLine(window, *m_settings.camera, vertices3[i], vertices3[(i + 1) % vertices3.size()], color);
					RenderSFMLImpl::renderLine(window, *m_settings.camera, vertices4[i], vertices4[(i + 1) % vertices4.size()], color);
				}
			}

			//std::vector<Vector2> curvatureOfCorner;

			//for (float i = 0.0f; i <= m_count; i += 1.0f)
			//{
			//	float angle = i * step;
			//	Vector2 start(radius * std::cos(angle), radius * std::sin(angle));
			//	Vector2 end(2.0f * radius * std::cos(angle), 2.0f * radius * std::sin(angle));
			//	start += center;
			//	end += center;
			//	RenderSFMLImpl::renderLine(window, *m_settings.camera, start, end, gray);
			//	curvatureOfCorner.push_back(end);
			//	if(i == 0.0f)
			//		continue;

			//	RenderSFMLImpl::renderLine(window, *m_settings.camera, curvatureOfCorner[i - 1], curvatureOfCorner[i], gray);
			//}
		}

		void onPostStep(real dt) override
		{
			
		}

		void onKeyPressed(sf::Event& event) override
		{

		}
		void onRenderUI() override
		{
			Vector2 pos(6.0f, -3.0f);
			pos = m_settings.camera->worldToScreen(pos);
			ImGui::SetNextWindowPos(ImVec2(pos.x, pos.y), ImGuiCond_Once);

			ImGui::Begin("Rounded Rect");

			ImGui::DragFloat("Half Width", &m_halfWidth, 0.01f, 0.5f, 50.0f);
			ImGui::DragFloat("Half Height", &m_halfHeight, 0.01f, 0.5f, 50.0f);
			ImGui::DragFloat("Rounded Radius Percentage", &m_percentage, 0.01f, 0.1f, 1.0f);
			ImGui::DragFloat("Inner Radius Percentage", &m_innerRadiusPercentage, 0.01f, 0.1f, 0.9f);
			ImGui::DragFloat("Corner Percentage Angle", &m_cornerPercentage, 0.01f, 0.2f, 0.6f);
			ImGui::DragFloat("Bezier Sample Counts", &m_bezierCount, 1, 8.0f, 100.0f);
			ImGui::DragFloat("Curvature Scale Factor", &m_curvatureScaleFactor, 0.01f, 0.1f, 1.0f);
			ImGui::DragFloat("Smoothness", &m_smoothness, 0.001f, 0.01f, 0.9f);
			ImGui::DragFloat("Segment Count", &m_count, 1, 4.0f, 100.0f);

			ImGui::Checkbox("Show Rounded Curvature", &m_showRoundedCurvature);
			ImGui::Checkbox("Show Bezier Curvature", &m_showBezierCurvature);
			ImGui::Checkbox("Show Reference Line", &m_showReferenceLine);
			ImGui::Checkbox("Show G1 Continuity", &m_showG1);
			ImGui::Checkbox("Show G2 Continuity", &m_showG2);

			ImGui::End();
		}
		void onUnLoad() override
		{

		}

		void onMousePress(sf::Event& event) override
		{
			if (event.mouseButton.button == sf::Mouse::Left)
			{
				if((m_mousePos - m_bezierPoints1[1]).length() < 0.05f)
				{
					m_currentIndex = 1;
					m_isMoving = true;
				}
				else if((m_mousePos - m_bezierPoints1[2]).length() < 0.05f)
				{
					m_currentIndex = 2;
					m_isMoving = true;
				}
				else if((m_mousePos - m_bezierPoints1[3]).length() < 0.05f)
				{
					m_currentIndex = 3;
					m_isMoving = true;
				}
				else
				{
					m_currentIndex = -1;
					m_isMoving = false;
				}
			}
		}
		void onMouseRelease(sf::Event& event) override
		{
			m_currentIndex = -1;
			m_isMoving = false;
		}
		void onMouseMove(sf::Event& event) override
		{
			Vector2 pos(static_cast<real>(event.mouseMove.x), static_cast<real>(event.mouseMove.y));
			m_mousePos = m_settings.camera->screenToWorld(pos);
			if (m_isMoving && m_currentIndex != -1)
			{
				if(m_currentIndex == 1)
				{
					m_bezierPoints1[1].x = m_mousePos.x;
				}
			}
			//std::cout << "mouse:(" << m_mousePos.x << "," << m_mousePos.y << ")\n";
			//std::cout << "bezier[1]:(" << m_bezierPoints[1].x << "," << m_bezierPoints[1].y << ")\n";
			//std::cout << "bezier[2]:(" << m_bezierPoints[2].x << "," << m_bezierPoints[2].y << ")\n";
		}

	private:
		bool m_showG1 = true;
		bool m_showG2 = true;
		bool m_showRoundedCurvature = false;
		bool m_showBezierCurvature = false;
		bool m_showReferenceLine = false;

		int m_currentIndex = -1;
		bool m_isMoving = false;
		Vector2 m_mousePos;
		Vector2 m_cornerCenter;

		Vector2 m_startRoundedPos;
		Vector2 m_endRoundedPos;

		std::array<Vector2, 4> m_bezierPoints1;

		std::array<Vector2, 4> m_bezierPoints2;

		float m_curvatureScaleFactor = 0.5f;
		float m_bezierCount = 50.0f;
		float m_count = 16;
		float m_halfWidth = 1.0f;
		float m_halfHeight = 1.0f;
		float m_smoothness = 0.5f;
		float m_percentage = 0.7f;
		float m_innerRadiusPercentage = 0.8f;
		float m_cornerPercentage = 0.2f;
		sf::Color color = RenderConstant::Cyan;
		sf::Color gray = sf::Color(158, 158, 158, 255);
		
	};
}
#endif
