#ifndef PHYSICS2D_SCENES_SENSOR_H
#define PHYSICS2D_SCENES_SENSOR_H
#include "./include/frame.h"
namespace Physics2D
{
	class SensorFrame : public Frame
	{
	public:
		SensorFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Camera* camera) : Frame("Sensor", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{
			edge.set({ -100, 0 }, { 100, 0 });
			rectangle.set(1.0f, 1.0f);
			circle.setRadius(10.0f);

			Body* ground;

			ground = m_world->createBody();
			ground->setShape(&edge);
			ground->position().set({ 0.0, 0.0 });
			ground->setMass(Constant::Max);
			ground->setType(Body::BodyType::Static);
			m_tree->insert(ground);

			real offset = 0.0f;
			real max = 25.0f;
			for (real j = 0; j < max; j += 1.0f)
			{
				Body* body = m_world->createBody();
				body->position().set({ 0.0f, 30.0f + j * 4.0f});
				body->setShape(&rectangle);
				body->rotation() = 0;
				body->setMass(0.2f);
				body->setType(Body::BodyType::Dynamic);
				body->setFriction(0.8f);
				body->setRestitution(0.0f);
				m_tree->insert(body);
			}

			for (real j = 0; j < max; j += 1.0f)
			{
				Body* body = m_world->createBody();
				body->position().set({ 10.0f, 32.5f + j * 4.0f });
				body->setShape(&rectangle);
				body->rotation() = 0;
				body->setMass(0.2f);
				body->setType(Body::BodyType::Dynamic);
				body->setFriction(0.8f);
				body->setRestitution(0.0f);
				m_tree->insert(body);
			}

			sensorRegion.rotation = Math::degreeToRadian(-45);
			sensorRegion.shape = &circle;
			sensorRegion.transform.set(5.0f, 15.0f);
		}
		void render(sf::RenderWindow& window) override
		{
			RenderSFMLImpl::renderShape(window, *m_camera, sensorRegion, sf::Color::Cyan);
		}
		void update(real dt)override
		{
			auto bodyList = m_tree->query(AABB::fromShape(sensorRegion));
			for (auto& body : bodyList)
			{
				ShapePrimitive primitive;
				primitive.rotation = body->rotation();
				primitive.shape = body->shape();
				primitive.transform = body->position();
				if (Detector::collide(primitive, sensorRegion))
					body->forces() += (sensorRegion.transform - body->position()).normal() * 30.0f;
			}
		}
	private:
		Rectangle rectangle;
		Edge edge;
		Circle circle;
		ShapePrimitive sensorRegion;

	};
}
#endif