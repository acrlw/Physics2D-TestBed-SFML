#ifndef PHYSICS2D_SCENES_DOMINO_H
#define PHYSICS2D_SCENES_DOMINO_H
#include "frame.h"
namespace Physics2D
{
	class DominoFrame : public Frame
	{
	public:

		DominoFrame(PhysicsSystem* system, Camera* camera) : Frame("Continuous", system, camera)
		{

		}
		void load() override
		{

			floor.set(15.0f, 0.5f);
			rectangle.set(0.5f, 0.5f);
			brick.set(0.3f, 3.0f);
			edge.set(Vec2{ -100.0f, 0 }, Vec2{ 100.0f, 0 });

			//Body* ground = m_world->createBody();
			//ground->setShape(&edge);
			//ground->setType(BodyType::Static);
			//ground->setMass(Constant::PosInfty);
			//ground->position().set({ 0, 0.0f });
			//ground->setFriction(0.1f);
			//ground->setRestitution(0.0f);
			//m_tree->insert(ground);

			//Body* tile = m_world->createBody();
			//tile->setShape(&floor);
			//tile->setType(BodyType::Static);
			//tile->setMass(Constant::PosInfty);
			//tile->setFriction(0.1f);
			//tile->setRestitution(0.0f);
			//tile->rotation() = degreeToRadian(15);
			//tile->position().set({ 4, 8 });
			//m_tree->insert(tile);

			//tile = m_world->createBody();
			//tile->setShape(&floor);
			//tile->setType(BodyType::Static);
			//tile->setMass(Constant::PosInfty);
			//tile->setFriction(0.1f);
			//tile->setRestitution(0.0f);
			//tile->rotation() = degreeToRadian(-15);
			//tile->position().set({ -4, 4 });
			//m_tree->insert(tile);


			//tile = m_world->createBody();
			//tile->setShape(&floor);
			//tile->setType(BodyType::Static);
			//tile->setMass(Constant::PosInfty);
			//tile->setFriction(0.1f);
			//tile->setRestitution(0.0f);
			//tile->rotation() = 0;
			//tile->position().set({ -5, 10 });
			//m_tree->insert(tile);

			//for(real i = 0;i < 9.0; i += 1.0f)
			//{
			//	Body* card = m_world->createBody();
			//	card->setShape(&brick);
			//	card->setMass(5.0f);
			//	card->setFriction(0.1f);
			//	card->setRestitution(0);
			//	card->setType(BodyType::Dynamic);
			//	card->position().set({ -10.0f + i * 1.2f, 12.0f });
			//	m_tree->insert(card);
			//}

			//Body* stammer = m_world->createBody();
			//stammer->setShape(&rectangle);
			//stammer->setMass(2.0f);
			//stammer->setFriction(0.1f);
			//stammer->setType(BodyType::Dynamic);
			//stammer->position().set(-16.0f, 16.0f);
			//m_tree->insert(stammer);

			//DistanceJointPrimitive djp;
			//djp.bodyA = stammer;
			//djp.localPointA.set(0, 0);
			//djp.minDistance = 1.0f;
			//djp.maxDistance = 4.0f;
			//djp.targetPoint.set(-12.0f, 16.0f);
			//m_world->createJoint(djp);

			//OrientationJointPrimitive ojp;
			//ojp.targetPoint.set(-12.0f, 16.0f);
			//ojp.bodyA = stammer;
			//ojp.referenceRotation = 0;
			//m_world->createJoint(ojp);



		}
		void render(sf::RenderWindow& window) override
		{

		}
	private:
		Rectangle brick;
		Rectangle floor;
		Edge edge;
		Rectangle rectangle;
	};
}
#endif