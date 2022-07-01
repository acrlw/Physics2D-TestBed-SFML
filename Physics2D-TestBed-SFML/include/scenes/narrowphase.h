#ifndef PHYSICS2D_SCENES_NARROWPHASE_H
#define PHYSICS2D_SCENES_NARROWPHASE_H
#include "./include/frame.h"

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

			shape1.shape = &polygon1;
			shape1.transform.set(1.0f, 2.0f);
			shape1.rotation = Math::degreeToRadian(30);

			shape2.shape = &polygon2;
			shape2.transform.set(1.0f, -5.0f);
			shape2.rotation = Math::degreeToRadian(30);
			result = Detector::detect(shape1, shape2);

		}
		void render(sf::RenderWindow& window) override
		{
			RenderSFMLImpl::renderShape(window, *m_camera, shape1, sf::Color::Green);
			RenderSFMLImpl::renderShape(window, *m_camera, shape2, sf::Color::Cyan);
			for (auto& elem : result.contactList)
			{
				RenderSFMLImpl::renderPoint(window, *m_camera, elem.pointA, RenderConstant::materialRed);
				RenderSFMLImpl::renderPoint(window, *m_camera, elem.pointB, RenderConstant::materialBlue);
			}
			RenderSFMLImpl::renderPoint(window, *m_camera, shape1.transform, sf::Color::Green);
			RenderSFMLImpl::renderPoint(window, *m_camera, shape2.transform, sf::Color::Cyan);
			RenderSFMLImpl::renderLine(window, *m_camera, shape1.transform, result.normal + shape1.transform, sf::Color::Green);
			RenderSFMLImpl::renderLine(window, *m_camera, shape2.transform, -result.normal + shape2.transform, sf::Color::Cyan);
		}
	private:
		Polygon polygon1;
		Polygon polygon2;
		ShapePrimitive shape1, shape2;
		Collision result;
	};
}
#endif