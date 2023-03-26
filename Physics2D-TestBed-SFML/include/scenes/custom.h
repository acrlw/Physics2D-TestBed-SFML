#ifndef PHYSICS2D_SCENES_CUSTOM_H
#define PHYSICS2D_SCENES_CUSTOM_H
#include "frame.h"
namespace Physics2D
{
	class CustomFrame : public Frame
	{
	public:
		CustomFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, UniformGrid* grid, Camera* camera) : Frame("Custom", world, maintainer, tree, grid, camera)
		{

		}
		void load() override
		{

			rectangle.set(0.5f, 0.5f);

		}
		void render(sf::RenderWindow& window) override
		{

		}
		void update(real dt) override
		{

		}
	private:

		Rectangle rectangle;
	};
}
#endif