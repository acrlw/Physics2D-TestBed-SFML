#ifndef PHYSICS2D_SCENES_XPBD_H
#define PHYSICS2D_SCENES_XPBD_H
#include "./include/frame.h"
namespace Physics2D
{
	class XPBDFrame : public Frame
	{
	public:
		XPBDFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Camera* camera) : Frame("Extended Position-Based Dynamics", world, maintainer, tree, dbvh, camera)
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
		Vector2 positions[10];
		Vector2 velocities[10];
		Vector2 masses[10];
		real distance = 1;
		real d;
		real k;
	};
}
#endif