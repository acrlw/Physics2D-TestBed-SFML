#ifndef PHYSICS2D_SCENES_BITMASK_H
#define PHYSICS2D_SCENES_BITMASK_H
#include "frame.h"
namespace Physics2D
{
	class BitmaskFrame : public Frame
	{
	public:
		BitmaskFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, UniformGrid* grid, Camera* camera) : Frame("Bitmask", world, maintainer, tree, grid, camera)
		{
			
		}
		void load() override
		{
			block.set(20, 1.0f);
			rectangle.set(1.0f, 1.0f);
			edge.set(Vector2{ -10.0f, 0.0f }, Vector2{ 10.0f, 0.0f });
			
			uint32_t mask = 0x01;
			real max = 1.0f;
			for(real i = 0;i < max;i += 1.0f)
			{
				Body* ground = m_world->createBody();
				ground->setShape(&edge);
				ground->position().set({ 0, 0 + i * 3.0f });
				ground->setFriction(0.4f);
				ground->setBitmask(mask);
				ground->setRestitution(0);
				ground->setMass(Constant::Max);
				ground->setType(Body::BodyType::Static);
				mask = mask << 1;
				m_tree->insert(ground);
			}
			mask = 0x01;
			for (real i = 0; i < max; i += 1.0f)
			{
				Body* body = m_world->createBody();
				body->setShape(&rectangle);
				body->position().set({ i * 3.0f + 0.5f, 0.5f });
				body->setFriction(0.9f);
				body->setBitmask(mask);
				body->setRestitution(0);
				body->setMass(1);
				body->setType(Body::BodyType::Dynamic);
				mask = mask << 1;
				m_tree->insert(body);
			}
		}
		void render(sf::RenderWindow& window) override
		{
			
		}
	private:
		Rectangle rectangle;
		Rectangle block;
		Edge edge;
	};
}
#endif