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
			block.set(50, 2.0f);
			rectangle.set(1.0f, 1.0f);
			edge.set(Vector2{ -10.0f, 0.0f }, Vector2{ 10.0f, 0.0f });

			uint32_t mask = 0x01;
			real max = 1.0f;
			for (real i = 0; i < max; i += 1.0f)
			{
				Body* ground = m_world->createBody();
				ground->setShape(&block);
				ground->position().set({ 0, -1.0f + i * 3.0f });
				ground->setFriction(0.6f);
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
				body = m_world->createBody();
				body->setShape(&rectangle);
				body->position().set({ i * 3.0f + 0.5f, 0.5f });
				body->setFriction(0.6f);
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
			ShapePrimitive sp;
			sp.transform.position = body->lastPosition();
			sp.transform.rotation = body->lastRotation();
			sp.shape = body->shape();
			RenderSFMLImpl::renderShape(window, *m_camera, sp, RenderConstant::MaterialGray);

		}
	private:
		Rectangle rectangle;
		Rectangle block;
		Edge edge;
		Body* body;

	};
}
#endif