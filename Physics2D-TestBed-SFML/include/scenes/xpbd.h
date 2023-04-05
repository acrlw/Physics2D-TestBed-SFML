#ifndef PHYSICS2D_SCENES_XPBD_H
#define PHYSICS2D_SCENES_XPBD_H
#include "frame.h"
namespace Physics2D
{
	class XPBDFrame : public Frame
	{
	public:
		XPBDFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, UniformGrid* grid, Camera* camera) : Frame("Extended Position-Based Dynamics", world, maintainer, tree, grid, camera)
		{

		}
		void load() override
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