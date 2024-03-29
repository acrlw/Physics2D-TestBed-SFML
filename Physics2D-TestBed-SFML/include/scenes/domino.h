#ifndef PHYSICS2D_SCENES_DOMINO_H
#define PHYSICS2D_SCENES_DOMINO_H
#include "frame.h"

namespace Physics2D
{
	class DominoFrame : public Frame
	{
	public:
		DominoFrame(const FrameSettings& settings) : Frame(settings)
		{
		}

		void onLoad() override
		{
			block.set(200, 1.0f);
			floor.set(15.0f, 0.8f);
			rectangle.set(0.5f, 0.5f);
			brick.set(0.35f, 2.5f);
			edge.set(Vector2{-100.0f, 0}, Vector2{100.0f, 0});

			Body* ground = m_settings.world->createBody();
			ground->setShape(&edge);
			ground->setType(Body::BodyType::Static);
			ground->setMass(Constant::Max);
			ground->position().set({0, 0.0f});
			ground->setFriction(0.1f);
			ground->setRestitution(0.0f);
			m_settings.tree->insert(ground);

			Body* tile = m_settings.world->createBody();
			tile->setShape(&floor);
			tile->setType(Body::BodyType::Static);
			tile->setMass(Constant::Max);
			tile->setFriction(0.1f);
			tile->setRestitution(0.0f);
			tile->rotation() = Math::degreeToRadian(20);
			tile->position().set({4, 10});
			m_settings.tree->insert(tile);

			tile = m_settings.world->createBody();
			tile->setShape(&floor);
			tile->setType(Body::BodyType::Static);
			tile->setMass(Constant::Max);
			tile->setFriction(0.1f);
			tile->setRestitution(0.0f);
			tile->rotation() = Math::degreeToRadian(-20);
			tile->position().set({-4, 4});
			m_settings.tree->insert(tile);


			tile = m_settings.world->createBody();
			tile->setShape(&floor);
			tile->setType(Body::BodyType::Static);
			tile->setMass(Constant::Max);
			tile->setFriction(0.2f);
			tile->setRestitution(0.0f);
			tile->rotation() = 0;
			tile->position().set({-5, 13});
			m_settings.tree->insert(tile);

			for (real i = 0; i < 13.0; i += 1.0f)
			{
				Body* card = m_settings.world->createBody();
				card->setShape(&brick);
				card->setMass(1.5f);
				card->setFriction(0.5f);
				card->setRestitution(0);
				card->setType(Body::BodyType::Dynamic);
				card->position().set({-9.8f + i * 1.0f, 15.0f});
				m_settings.tree->insert(card);
			}

			Body* stammer = m_settings.world->createBody();
			stammer->setShape(&rectangle);
			stammer->setMass(10.0f);
			stammer->setFriction(0.1f);
			stammer->setType(Body::BodyType::Dynamic);
			stammer->position().set(-16.0f, 19.5f);
			m_settings.tree->insert(stammer);

			DistanceJointPrimitive djp;
			djp.bodyA = stammer;
			djp.bodyB = ground;
			djp.localPointA.set(0, 0);
			djp.localPointB.set(-12.0f, 19.5f);
			djp.minDistance = 4.0f;
			djp.maxDistance = 4.0f;
			m_settings.world->createJoint(djp);

			OrientationJointPrimitive ojp;
			ojp.targetPoint.set(-12.0f, 19.5f);
			ojp.bodyA = stammer;
			ojp.referenceRotation = 0;
			m_settings.world->createJoint(ojp);

			//std::cout << "size:" << m_settings.world->bodyList().size() << std::endl;
		}

		void onPostRender(sf::RenderWindow& window) override
		{
			//std::cout << "size:" << m_settings.world->bodyList().size() << std::endl;
		}

	private:
		Rectangle block;
		Rectangle brick;
		Rectangle floor;
		Edge edge;
		Rectangle rectangle;
	};
}
#endif
