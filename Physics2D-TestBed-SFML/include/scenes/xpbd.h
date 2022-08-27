#ifndef PHYSICS2D_SCENES_XPBD_H
#define PHYSICS2D_SCENES_XPBD_H
#include "./include/frame.h"
namespace Physics2D
{
	class XPBDFrame : public Frame
	{
	public:
		XPBDFrame(PhysicsSystem* system, Camera* camera) : Frame("Extended Position-Based Dynamics", system, camera)
		{

		}
		void load() override
		{

		}
		void render(sf::RenderWindow& window) override
		{

		}
		void update(real dt) override
		{

		}
	private:
		Vec2 positions[10];
		Vec2 velocities[10];
		Vec2 masses[10];
		real distance = 1;
		real d;
		real k;
	};
}
#endif