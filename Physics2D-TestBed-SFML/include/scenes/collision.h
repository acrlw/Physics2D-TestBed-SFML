#ifndef PHYSICS2D_SCENES_COLLISION_H
#define PHYSICS2D_SCENES_COLLISION_H
#include "./include/frame.h"
namespace Physics2D
{
	class CollisionFrame : public Frame
	{
	public:
		CollisionFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Camera* camera) : Frame("Collision", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{
			edge.set({ -10, 0 }, { 10, 0 });
			capsule.set(2.0f, 1.0f);
			sector.set(0.0f, 2.0f * Constant::Pi / 3.0f, 2.0f);
			rectangle.set(1.0f, 1.0f);

			Body* ground;
			Body* rect;

			ground = m_world->createBody();
			ground->setShape(&edge);
			ground->position().set({ 0, 0 });
			ground->setMass(Constant::PosInfty);
			ground->setType(Body::BodyType::Static);
			ground->setFriction(0.7f);
			ground->setRestitution(1.0);
			m_tree->insert(ground);

			rect = m_world->createBody();
			rect->setShape(&rectangle);
			rect->position().set({ 0, 6 });
			rect->rotation() = degreeToRadian(60);
			rect->setMass(1);
			rect->setType(Body::BodyType::Dynamic);
			rect->setFriction(0.4f);
			rect->setRestitution(0.0f);
			m_tree->insert(rect);
		}
		void render(sf::RenderWindow& window) override
		{

		}
	private:
		Capsule capsule;
		Edge edge;
		Sector sector;
		Rectangle rectangle;

	};
}
#endif