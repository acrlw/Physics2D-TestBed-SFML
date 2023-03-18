#ifndef PHYSICS2D_SCENES_GEOMETRY_H
#define PHYSICS2D_SCENES_GEOMETRY_H
#include "frame.h"

namespace Physics2D
{
	class GeometryFrame : public Frame
	{
	public:

		GeometryFrame(PhysicsSystem* system, Camera* camera) : Frame("Geometry", system, camera)
		{

		}
		void load() override
		{
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_real_distribution<> dist1(-20.0f, 5.0f);
			std::uniform_real_distribution<> dist2(-5.0f, 20.0f);

			for (int i = 0; i < 500; i++)
				points1.emplace_back(Vec2(dist1(gen), dist1(gen)));

			convex1 = GeometryAlgorithm2D::grahamScan(points1);

			for (int i = 0; i < 500; i++)
				points2.emplace_back(Vec2(dist2(gen), dist2(gen)));

			convex2 = GeometryAlgorithm2D::grahamScan(points2);
			intersectionConvex = GeometryAlgorithm2D::Clipper::sutherlandHodgmentPolygonClipping(convex1, convex2);

		}
		void render(sf::RenderWindow& window) override
		{
			for (int i = 0; i < convex1.size() - 1; i++)
				RenderSFMLImpl::renderLine(window, *m_camera, convex1[i], convex1[i + 1], sf::Color::Green);

			for (int i = 0; i < convex2.size() - 1; i++)
				RenderSFMLImpl::renderLine(window, *m_camera, convex2[i], convex2[i + 1], RenderConstant::MaterialBlue);

			for (int i = 0; i < intersectionConvex.size() - 1; i++)
				RenderSFMLImpl::renderLine(window, *m_camera, intersectionConvex[i], intersectionConvex[i + 1], sf::Color::Yellow);


			RenderSFMLImpl::renderPoints(window, *m_camera, points1, sf::Color::Cyan);
			RenderSFMLImpl::renderPoints(window, *m_camera, points2, sf::Color::Magenta);
		}
	private:
		Container::Vector<Vec2> points1;
		Container::Vector<Vec2> points2;
		Container::Vector<Vec2> convex1;
		Container::Vector<Vec2> convex2;
		Container::Vector<Vec2> intersectionConvex;
	};
}
#endif