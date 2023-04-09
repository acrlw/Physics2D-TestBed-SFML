#ifndef PHYSICS2D_SCENES_JOINTS_H
#define PHYSICS2D_SCENES_JOINTS_H
#include "frame.h"

namespace Physics2D
{
	class JointsFrame : public Frame
	{
	public:
		JointsFrame(const FrameSettings& settings) : Frame(settings)
		{
		}

		void onLoad() override
		{
			capsule.set(1.5f, 0.5f);
			triangle.append({{-1.0f, 1.0f}, {0.0f, -2.0f}, {1.0f, -1.0f}});
			triangle.scale(0.5f);
			edge.set({-100, 0}, {100, 0});
			edge2.set({0, 0}, {100, 40});
			rectangle.set(1.0f, 1.0f);

			wheel.setRadius(1.0f);

			uint32_t bitmask = 0x01;

			bodyA = m_settings.world->createBody();
			bodyA->setShape(&rectangle);
			bodyA->setMass(1.0f);
			bodyA->setType(Body::BodyType::Dynamic);
			bodyA->position().set(2.00, 0.0f);
			bodyA->setBitmask(0x01);
			m_settings.tree->insert(bodyA);

			bodyB = m_settings.world->createBody();
			bodyB->setShape(&rectangle);
			bodyB->setMass(1.0f);
			bodyB->setType(Body::BodyType::Dynamic);
			bodyB->position().set(0, 0.0f);
			bodyB->setBitmask(0x01);
			m_settings.tree->insert(bodyB);

			WeldJointPrimitive rjp;
			rjp.bodyA = bodyA;
			rjp.bodyB = bodyB;
			rjp.dampingRatio = 0.8f;
			rjp.frequency = 10;
			rjp.maxForce = 10000;
			rjp.localPointA.set(-0.5, 0.5);
			rjp.localPointB.set(0.5, 0.5);

			joint = m_settings.world->createJoint(rjp);

			updateJoint();

			RotationJointPrimitive rotationPrim;
			rotationPrim.bodyA = bodyA;
			rotationPrim.bodyB = bodyB;
			rotationPrim.referenceRotation = Math::degreeToRadian(30);
			m_settings.world->createJoint(rotationPrim);

			block = m_settings.world->createBody();
			block->setShape(&rectangle);
			block->setType(Body::BodyType::Dynamic);
			block->setMass(4.0f);
			block->position().set(5.0f, 2.0f);
			block->setBitmask(0x01 << 1);
			m_settings.tree->insert(block);

			wheel1 = m_settings.world->createBody();
			wheel2 = m_settings.world->createBody();
			wheel1->setShape(&wheel);
			wheel2->setShape(&wheel);
			wheel1->setMass(4.0f);
			wheel2->setMass(4.0f);
			wheel1->position().set(3.0f, 1.0f);
			wheel2->position().set(7.0f, 1.0f);
			wheel1->setType(Body::BodyType::Dynamic);
			wheel2->setType(Body::BodyType::Dynamic);
			wheel1->setBitmask(0x01 << 2 | 0x01);
			wheel2->setBitmask(0x01 << 3 | 0x01);
			m_settings.tree->insert(wheel1);
			m_settings.tree->insert(wheel2);

			WeldJointPrimitive rjp1, rjp2;
			rjp1.bodyA = wheel1;
			rjp2.bodyA = wheel2;
			rjp1.bodyB = block;
			rjp2.bodyB = block;
			rjp1.localPointA.clear();
			rjp2.localPointA.clear();
			rjp1.localPointB.set(-2.0f, -1.0f);
			rjp2.localPointB.set(2.0f, -1.0f);
			m_settings.world->createJoint(rjp1);
			m_settings.world->createJoint(rjp2);

			Body* ground = m_settings.world->createBody();
			ground->setShape(&edge);
			ground->position().set({0, -2.0});
			ground->setMass(Constant::Max);
			ground->setType(Body::BodyType::Static);
			ground->setFriction(0.4f);
			ground->setBitmask(0x01);
			m_settings.tree->insert(ground);

			ground = m_settings.world->createBody();
			ground->setShape(&edge2);
			ground->position().set(100.0f, -2.0f);
			ground->setMass(Constant::Max);
			ground->setType(Body::BodyType::Static);
			ground->setFriction(0.4f);
			ground->setBitmask(0x01);
			m_settings.tree->insert(ground);
		}

		void onPostRender(sf::RenderWindow& window) override
		{
			sf::Color color = sf::Color::Cyan;
			color.a = 155;
			Vector2 p = bodyA->toWorldPoint(joint->primitive().localPointA);
			RenderSFMLImpl::renderLine(window, *m_settings.camera, p - 0.5f * distance * normal, p + 0.5f * distance * normal,
			                           color);
		}

		void onUnLoad() override
		{
			m_settings.camera->setTargetBody(nullptr);
		}

		void onPostStep(real dt) override
		{
			updateJoint();
		}

		void onKeyPressed(sf::Event& event) override
		{
			switch (event.key.code)
			{
			case sf::Keyboard::D:
				{
					wheel1->applyImpulse(Vector2(0.0f, -50.0f), Vector2(1.0f, 0.0f));
					break;
				}
			default:
				break;
			}
		}

	private:
		void updateJoint()
		{
			//auto pair = Detector::distance(bodyA, bodyB);
			//normal = (pair.pointA - pair.pointB).normalize();
			//Vector2 pb = pair.pointB + 0.5f * distance * normal;
			//Vector2 pa = pair.pointA - 0.5f * distance * normal;
			//joint->primitive().localPointA = bodyA->toLocalPoint(pa);
			//joint->primitive().localPointB = bodyB->toLocalPoint(pb);
		}

		Capsule capsule;
		Polygon triangle;
		Rectangle rectangle;
		Circle wheel;

		Edge edge;
		Edge edge2;
		real distance = 2.0f;
		Vector2 normal;

		WeldJoint* joint;
		Body* bodyA;
		Body* bodyB;
		Body* block;
		Body* wheel1;
		Body* wheel2;
	};
}
#endif
