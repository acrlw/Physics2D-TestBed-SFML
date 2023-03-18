#ifndef PHYSICS2D_SCENES_SENSOR_H
#define PHYSICS2D_SCENES_SENSOR_H
#include "frame.h"
namespace Physics2D
{
	class SensorFrame : public Frame
	{
	public:
		SensorFrame(PhysicsSystem* system, Camera* camera) : Frame("Sensor", system, camera)
		{

		}
		void load() override
		{
			edge.set({ -100, 0 }, { 100, 0 });
			rectangle.set(1.0f, 1.0f);
			circle.setRadius(10.0f);

			//Body* ground;

			//ground = m_world->createBody();
			//ground->setShape(&edge);
			//ground->position().set({ 0.0, 0.0 });
			//ground->setMass(Constant::PosInfty);
			//ground->setType(BodyType::Static);
			//m_tree->insert(ground);

			//real offset = 0.0f;
			//real max = 25.0f;
			//for (real j = 0; j < max; j += 1.0f)
			//{
			//	Body* body = m_world->createBody();
			//	body->position().set({ 0.0f, 30.0f + j * 4.0f});
			//	body->setShape(&rectangle);
			//	body->rotation() = 0;
			//	body->setMass(0.2f);
			//	body->setType(BodyType::Dynamic);
			//	body->setFriction(0.5f);
			//	body->setRestitution(0.0f);
			//	m_tree->insert(body);
			//}

			//for (real j = 0; j < max; j += 1.0f)
			//{
			//	Body* body = m_world->createBody();
			//	body->position().set({ 10.0f, 32.5f + j * 4.0f });
			//	body->setShape(&rectangle);
			//	body->rotation() = 0;
			//	body->setMass(0.2f);
			//	body->setType(BodyType::Dynamic);
			//	body->setFriction(0.5f);
			//	body->setRestitution(0.0f);
			//	m_tree->insert(body);
			//}
			transform.rotation = degreeToRadian(-45);
			transform.position.set(5.0f, 5.0f);
		}
		void render(sf::RenderWindow& window) override
		{
			RenderSFMLImpl::renderShape(window, *m_camera, transform, &circle, sf::Color::Cyan);
		}
		void update(real dt)override
		{
			//auto bodyList = m_tree->query(AABB::fromShape(sensorRegion));
			//for (auto& body : bodyList)
			//{
			//	ShapeTransform primitive;
			//	primitive.rotation = body->rotation();
			//	primitive.shape = body->shape();
			//	primitive.transform = body->position();
			//	if (Detector::collide(primitive, sensorRegion))
			//		body->forces() += (sensorRegion.transform - body->position()).normal() * 30.0f;
			//}
		}
	private:
		Rectangle rectangle;
		Edge edge;
		Circle circle;
		Transform transform;

	};
}
#endif