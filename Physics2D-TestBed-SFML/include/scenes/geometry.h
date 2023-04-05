#ifndef PHYSICS2D_SCENES_GEOMETRY_H
#define PHYSICS2D_SCENES_GEOMETRY_H
#include "frame.h"

namespace Physics2D
{
	class GeometryFrame : public Frame
	{
	public:
		GeometryFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, UniformGrid* grid, Camera* camera) : Frame("Geometry", world, maintainer, tree, grid, camera)
		{

		}
		void load() override
		{
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_real_distribution<> dist1(-20.0f, 5.0f);
			std::uniform_real_distribution<> dist2(-5.0f, 20.0f);

			for (int i = 0; i < 500; i++)
				points1.emplace_back(Vector2(dist1(gen), dist1(gen)));

			convex1 = GeometryAlgorithm2D::grahamScan(points1);

			for (int i = 0; i < 500; i++)
				points2.emplace_back(Vector2(dist2(gen), dist2(gen)));

			convex2 = GeometryAlgorithm2D::grahamScan(points2);
			//convex1.emplace_back(*convex1.begin());
			//convex2.emplace_back(*convex2.begin());
			intersectionConvex = GeometryAlgorithm2D::Clipper::sutherlandHodgmentPolygonClipping(convex1, convex2);

		}
		void postRender(sf::RenderWindow& window) override
		{
			for (auto iter = convex1.begin(); iter != convex1.end(); ++iter)
			{
				auto next = iter + 1;
				if (next == convex1.end())
					next = convex1.begin();
				RenderSFMLImpl::renderLine(window, *m_camera, *iter, *next, sf::Color::Green);
			}

			for (auto iter = convex2.begin(); iter != convex2.end(); ++iter)
			{
				auto next = iter + 1;
				if (next == convex2.end())
					next = convex2.begin();
				RenderSFMLImpl::renderLine(window, *m_camera, *iter, *next, RenderConstant::MaterialBlue);
			}

			for (auto iter = intersectionConvex.begin(); iter != intersectionConvex.end(); ++iter)
			{
				auto next = iter + 1;
				if (next == intersectionConvex.end())
					next = intersectionConvex.begin();
				RenderSFMLImpl::renderLine(window, *m_camera, *iter, *next, sf::Color::Yellow);
			}

			RenderSFMLImpl::renderPoints(window, *m_camera, points1, sf::Color::Cyan);
			RenderSFMLImpl::renderPoints(window, *m_camera, points2, sf::Color::Magenta);
		}
	private:
		Container::Vector<Vector2> points1;
		Container::Vector<Vector2> points2;
		Container::Vector<Vector2> convex1;
		Container::Vector<Vector2> convex2;
		Container::Vector<Vector2> intersectionConvex;
	};
}
#endif