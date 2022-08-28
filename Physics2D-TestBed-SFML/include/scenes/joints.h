#ifndef PHYSICS2D_SCENES_JOINTS_H
#define PHYSICS2D_SCENES_JOINTS_H
#include "./include/frame.h"
namespace Physics2D
{
	class JointsFrame : public Frame
	{
	public:
		JointsFrame(PhysicsSystem* system, Camera* camera) : Frame("Joints", system, camera)
		{

		}
		void load() override
		{
			capsule.set(1.5f, 0.5f);
			triangle.append({ {-1.0f, 1.0f},{0.0f, -2.0f},{1.0f, -1.0f},{-1.0f, 1.0f} });
			triangle.scale(0.5f);
			edge.set({ -100, 0 }, { 100, 0 });
			edge2.set({ 0, 0 }, { 100, 40 });
			rectangle.set(4.0f, 2.0f);
			path.set(Vec2{ -6.0f, 6.0f }, Vec2{ -3.0f, 3.0f }, Vec2{ 3.0f, 9.0f }, Vec2{ 6.0f, 6.0f });
			wheel.setRadius(1.0f);

			//uint32_t bitmask = 0x01;

			//bodyA = m_world->createBody();
			//bodyA->setShape(&triangle);
			//bodyA->setMass(1.0f);
			//bodyA->setType(BodyType::Dynamic);
			//bodyA->position().set(0, 2.0f);
			//bodyA->setBitmask(0x01);
			//m_tree->insert(bodyA);

			//bodyB = m_world->createBody();
			//bodyB->setShape(&capsule);
			//bodyB->setMass(1.0f);
			//bodyB->setType(BodyType::Dynamic);
			//bodyB->position().set(0, 0.0f);
			//bodyB->setBitmask(0x01);
			//m_tree->insert(bodyB);

			//RevoluteJointPrimitive rjp;
			//rjp.bodyA = bodyA;
			//rjp.bodyB = bodyB;
			//rjp.dampingRatio = 0.8f;
			//rjp.frequency = 10;
			//rjp.maxForce = 10000;
			//
			//joint = m_world->createJoint(rjp);

			//updateJoint();

			//RotationJointPrimitive rotationPrim;
			//rotationPrim.bodyA = bodyA;
			//rotationPrim.bodyB = bodyB;
			//rotationPrim.referenceRotation = degreeToRadian(30);
			//m_world->createJoint(rotationPrim);

			//block = m_world->createBody();
			//block->setShape(&rectangle);
			//block->setType(BodyType::Dynamic);
			//block->setMass(4.0f);
			//block->position().set(5.0f, 2.0f);
			//block->setBitmask(0x01 << 1);
			//m_tree->insert(block);

			//wheel1 = m_world->createBody();
			//wheel2 = m_world->createBody();
			//wheel1->setShape(&wheel);
			//wheel2->setShape(&wheel);
			//wheel1->setMass(4.0f);
			//wheel2->setMass(4.0f);
			//wheel1->position().set(3.0f, 1.0f);
			//wheel2->position().set(7.0f, 1.0f);
			//wheel1->setType(BodyType::Dynamic);
			//wheel2->setType(BodyType::Dynamic);
			//wheel1->setBitmask(0x01 << 2 | 0x01);
			//wheel2->setBitmask(0x01 << 3 | 0x01);
			//m_tree->insert(wheel1);
			//m_tree->insert(wheel2);

			//RevoluteJointPrimitive rjp1, rjp2;
			//rjp1.bodyA = wheel1;
			//rjp2.bodyA = wheel2;
			//rjp1.bodyB = block;
			//rjp2.bodyB = block;
			//rjp1.localPointA.clear();
			//rjp2.localPointA.clear();
			//rjp1.localPointB.set(-2.0f, -1.0f);
			//rjp2.localPointB.set(2.0f, -1.0f);
			//m_world->createJoint(rjp1);
			//m_world->createJoint(rjp2);

			//Body* ground = m_world->createBody();
			//ground->setShape(&edge);
			//ground->position().set({ 0, -2.0 });
			//ground->setMass(Constant::PosInfty);
			//ground->setType(BodyType::Static);
			//ground->setFriction(0.4f);
			//ground->setBitmask(0x01);
			//m_tree->insert(ground);

			//ground = m_world->createBody();
			//ground->setShape(&edge2);
			//ground->position().set(100.0f, -2.0f);
			//ground->setMass(Constant::PosInfty);
			//ground->setType(BodyType::Static);
			//ground->setFriction(0.4f);
			//ground->setBitmask(0x01);
			//m_tree->insert(ground);

		}
		void render(sf::RenderWindow& window) override
		{
			//sf::Color color = sf::Color::Cyan;
			//color.a = 155;
			//Vec2 p = bodyA->toWorldPoint(joint->primitive().localPointA);
			//RenderSFMLImpl::renderLine(window, *m_camera, p - 0.5f * distance * normal, p + 0.5f * distance * normal, color);

		}
		void release() override
		{
			m_camera->setTargetBody(nullptr);
		}
		void update(real dt) override
		{
			updateJoint();
		}
		void onKeyPressed(sf::Event& event) 
		{
			switch (event.key.code)
			{
			case sf::Keyboard::D:
			{
				//wheel1->applyImpulse(Vec2(0.0f, -50.0f), Vec2(1.0f, 0.0f));
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
			//Vec2 pb = pair.pointB + 0.5f * distance * normal;
			//Vec2 pa = pair.pointA - 0.5f * distance * normal;
			//joint->primitive().localPointA = bodyA->toLocalPoint(pa);
			//joint->primitive().localPointB = bodyB->toLocalPoint(pb);

		}
		Capsule capsule;
		Polygon triangle;
		Rectangle rectangle;
		Circle wheel;
		Curve path;
		Edge edge;
		Edge edge2;
		real distance = 2.0f;
		Vec2 normal;

		RevoluteJoint* joint;
		Body* bodyA;
		Body* bodyB;
		Body* block; 
		Body* wheel1;
		Body* wheel2;
	};
}
#endif