#ifndef PHYSICS2D_SCENES_CUSTOM_H
#define PHYSICS2D_SCENES_CUSTOM_H
#include "frame.h"
namespace Physics2D
{
	class CustomFrame : public Frame
	{
	public:
		CustomFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, Camera* camera) : Frame("Custom", world, maintainer, tree, camera)
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

	};
}
#endif