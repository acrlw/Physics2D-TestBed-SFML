#ifndef PHYSICS2D_SCENES_PBD_H
#define PHYSICS2D_SCENES_PBD_H
#include "./include/frame.h"
namespace Physics2D
{
	class PBDFrame : public Frame
	{
	public:
		PBDFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Camera* camera) : Frame("Position-Based Dynamics", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{

		}
		void render(sf::RenderWindow& window) override
		{

		}
	private:

	};
}
#endif