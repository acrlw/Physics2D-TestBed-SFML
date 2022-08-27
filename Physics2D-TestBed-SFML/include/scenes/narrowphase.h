#ifndef PHYSICS2D_SCENES_NARROWPHASE_H
#define PHYSICS2D_SCENES_NARROWPHASE_H
#include "./include/frame.h"

namespace Physics2D
{
	class NarrowphaseFrame : public Frame
	{
	public:

		NarrowphaseFrame(PhysicsSystem* system, Camera* camera) : Frame("NarrowPhase", system, camera)
		{

		}
		void load() override
		{
			polygon1.append({ {0.0f, 4.0f},{-3.0f, 3.0f},{-4.0f, 0.0f},{-3.0f, -3.0f},{0, -4.0f},
			{3.0f, -3.0f}, {4.0f, 0.0f }, {3.0f, 3.0f },{0.0f, 4.0f } });

			polygon2.append({ {0.0f, 4.0f},{-3.0f, 3.0f},{-4.0f, 0.0f},{-3.0f, -3.0f},{0, -4.0f},
			{3.0f, -3.0f}, {4.0f, 0.0f }, {3.0f, 3.0f },{0.0f, 4.0f } });


			transform1.position.set(1.0f, 2.0f);
			transform1.rotation = degreeToRadian(30);


			transform2.position.set(1.0f, -5.0f);
			transform2.rotation = degreeToRadian(30);
			result = Detector::detect(transform1, &polygon1, transform2, &polygon2);

		}
		void render(sf::RenderWindow& window) override
		{
			RenderSFMLImpl::renderShape(window, *m_camera, transform1, &polygon1, sf::Color::Green);
			RenderSFMLImpl::renderShape(window, *m_camera, transform2, &polygon2, sf::Color::Cyan);
			for (auto& elem : result.contactList)
			{
				RenderSFMLImpl::renderPoint(window, *m_camera, elem.pointA, RenderConstant::MaterialRed);
				RenderSFMLImpl::renderPoint(window, *m_camera, elem.pointB, RenderConstant::MaterialBlue);
			}
			RenderSFMLImpl::renderPoint(window, *m_camera, transform1.position, sf::Color::Green);
			RenderSFMLImpl::renderPoint(window, *m_camera, transform2.position, sf::Color::Cyan);
			RenderSFMLImpl::renderLine(window, *m_camera, transform1.position, result.normal * result.penetration + transform1.position, sf::Color::Green);
			RenderSFMLImpl::renderLine(window, *m_camera, transform2.position, -result.normal * result.penetration + transform2.position, sf::Color::Cyan);
		}
	private:
		Polygon polygon1;
		Polygon polygon2;
		Transform transform1, transform2;
		Collision result;
	};
}
#endif