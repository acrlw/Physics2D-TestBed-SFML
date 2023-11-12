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
			bar.set(2.0f, 0.5f);

			wheel.setRadius(1.0f);

			Body* ground = m_settings.world->createBody();
			ground->setShape(&edge);
			ground->position().set({ 0, -2.0 });
			ground->setMass(Constant::Max);
			ground->setType(Body::BodyType::Static);
			ground->setFriction(0.4f);
			ground->setBitmask(0x01);
			m_settings.tree->insert(ground);

			uint32_t bitmask = 0x01;

			//bodyA = m_settings.world->createBody();
			//bodyA->setShape(&rectangle);
			//bodyA->setMass(1.0f);
			//bodyA->setType(Body::BodyType::Dynamic);
			//bodyA->position().set(0.00f, 0.0f);
			//bodyA->setBitmask(0x01);
			//m_settings.tree->insert(bodyA);

			//bodyB = m_settings.world->createBody();
			//bodyB->setShape(&rectangle);
			//bodyB->setMass(1.0f);
			//bodyB->setType(Body::BodyType::Dynamic);
			//bodyB->position().set(-2.0f, 2.0f);
			//bodyB->setBitmask(0x01);
			//m_settings.tree->insert(bodyB);

			//WeldJointPrimitive wjp;
			//wjp.bodyA = bodyA;
			//wjp.bodyB = bodyB;
			//wjp.dampingRatio = 0.8f;
			//wjp.frequency = 10;
			//wjp.maxForce = 10000;
			//wjp.referenceAngle = Math::degreeToRadian(-45);
			//wjp.localPointA.set(-0.55f, 0.55f);
			//wjp.localPointB.set(0.55f, -0.55f);

			//joint = m_settings.world->createJoint(wjp);

			//updateJoint();


			block = m_settings.world->createBody();
			block->setShape(&rectangle);
			block->setType(Body::BodyType::Dynamic);
			block->setMass(8.0f);
			block->position().set(2.5f, 1.0f);
			block->rotation() = Math::degreeToRadian(60);
			block->setBitmask(0x01);
			m_settings.tree->insert(block);

			//DistanceJointPrimitive djp;
			//djp.bodyA = block;
			//djp.bodyB = ground;
			//djp.localPointA.set(0.0f, 0.0f);
			//djp.localPointB.set(0.5f, 8.0f);
			//djp.minDistance = 4.0f;
			//djp.maxDistance = 4.0f;

			//DistanceJoint* dj = m_settings.world->createJoint(djp);

			//MotorJointPrimitive mjp;
			//mjp.bodyA = block;
			//mjp.bodyB = ground;
			//mjp.localPointA.set(0.0f, 0.0f);
			//mjp.localPointB.set(2.5f, 6.5f);
			//mjp.referenceAngle = Math::degreeToRadian(45);
			//mjp.correctionFactor = 0.4f;
			//MotorJoint* mj = m_settings.world->createJoint(mjp);
			
			//RevoluteJointPrimitive rjp;
			//rjp.bodyA = block;
			//rjp.bodyB = ground;
			//rjp.localPointA.set(0.0f, 0.0f);
			//rjp.localPointB.set(2.5f, 3.5f);
			//rjp.angularLimit = true;
			//rjp.referenceAngle = 0;
			//rjp.lowerAngle = Math::degreeToRadian(30);
			//rjp.upperAngle = Math::degreeToRadian(120);

			//RevoluteJoint* rj = m_settings.world->createJoint(rjp);

			PathJointPrimitive pjp;
			pjp.bodyA = block;
			pjp.origin.set(0.0f, 6.0f);
			pjp.localPointA.set(0.0f, 0.25f);

			PathJoint * pj = m_settings.world->createJoint(pjp);

			//PrismaticJointPrimitive pjp;
			//pjp.bodyA = block;
			//pjp.bodyB = ground;
			//pjp.localPointA.set(0.5f, 0.0f);
			//pjp.localPointB.set(2.5f, 3.5f);
			//pjp.xAxis.set(1.0f, 0.0f);
			//pjp.xAxis.normalize();
			//pjp.referenceAngle = Math::degreeToRadian(-90);
			//
			//pjp.lowerLimit = -2.0f;
			//pjp.upperLimit = 2.0f;
			//PrismaticJoint * pj = m_settings.world->createJoint(pjp);

			//wheel1 = m_settings.world->createBody();
			//wheel2 = m_settings.world->createBody();
			//wheel1->setShape(&wheel);
			//wheel2->setShape(&wheel);
			//wheel1->setMass(4.0f);
			//wheel2->setMass(4.0f);
			//wheel1->position().set(3.0f, 1.0f);
			//wheel2->position().set(7.0f, 1.0f);
			//wheel1->setType(Body::BodyType::Dynamic);
			//wheel2->setType(Body::BodyType::Dynamic);
			//wheel1->setBitmask(0x01 << 2 | 0x01);
			//wheel2->setBitmask(0x01 << 3 | 0x01);
			//m_settings.tree->insert(wheel1);
			//m_settings.tree->insert(wheel2);

			//RevoluteJointPrimitive rjp1, rjp2;
			//rjp1.bodyA = wheel1;
			//rjp2.bodyA = wheel2;
			//rjp1.bodyB = block;
			//rjp2.bodyB = block;
			//rjp1.localPointA.clear();
			//rjp2.localPointA.clear();
			//rjp1.localPointB.set(-2.0f, -1.0f);
			//rjp2.localPointB.set(2.0f, -1.0f);
			//m_settings.world->createJoint(rjp1);
			//m_settings.world->createJoint(rjp2);



			//ground = m_settings.world->createBody();
			//ground->setShape(&edge2);
			//ground->position().set(100.0f, -2.0f);
			//ground->setMass(Constant::Max);
			//ground->setType(Body::BodyType::Static);
			//ground->setFriction(0.4f);
			//ground->setBitmask(0x01);
			//m_settings.tree->insert(ground);
		}

		void onPostRender(sf::RenderWindow& window) override
		{


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
		Rectangle bar;
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
