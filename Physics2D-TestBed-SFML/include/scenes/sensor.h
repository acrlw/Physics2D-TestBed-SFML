#ifndef PHYSICS2D_SCENES_SENSOR_H
#define PHYSICS2D_SCENES_SENSOR_H
#include "frame.h"

namespace Physics2D
{
	class SensorFrame : public Frame
	{
	public:
		SensorFrame(const FrameSettings& settings) : Frame(settings)
		{
		}

		void onLoad() override
		{
			edge.set({-100, 0}, {100, 0});
			rectangle.set(1.0f, 1.0f);
			circle.setRadius(10.0f);

			Body* ground;

			ground = m_settings.world->createBody();
			ground->setShape(&edge);
			ground->position().set({0.0, 0.0});
			ground->setMass(Constant::Max);
			ground->setType(Body::BodyType::Static);
			m_settings.tree->insert(ground);

			real offset = 0.0f;
			real max = 25.0f;
			for (real j = 0; j < max; j += 1.0f)
			{
				Body* body = m_settings.world->createBody();
				body->position().set({-5.0f, 30.0f + j * 4.0f});
				body->setShape(&rectangle);
				body->rotation() = 0;
				body->setMass(1.0f);
				body->setType(Body::BodyType::Dynamic);
				body->setFriction(0.5f);
				body->setRestitution(0.0f);
				m_settings.tree->insert(body);
			}

			for (real j = 0; j < max; j += 1.0f)
			{
				Body* body = m_settings.world->createBody();
				body->position().set({5.0f, 32.5f + j * 4.0f});
				body->setShape(&rectangle);
				body->rotation() = 0;
				body->setMass(1.0f);
				body->setType(Body::BodyType::Dynamic);
				body->setFriction(0.5f);
				body->setRestitution(0.0f);
				m_settings.tree->insert(body);
			}

			sensorRegion.transform.rotation = Math::degreeToRadian(-45);
			sensorRegion.shape = &circle;
			sensorRegion.transform.position.set(0.0f, 15.0f);
		}

		void onPostRender(sf::RenderWindow& window) override
		{
			RenderSFMLImpl::renderShape(window, *m_settings.camera, sensorRegion, sf::Color::Cyan);
		}

		void onPostStep(real dt) override
		{
			auto bodyList = m_settings.tree->query(AABB::fromShape(sensorRegion));
			for (auto& body : bodyList)
			{
				ShapePrimitive primitive;
				primitive.transform.rotation = body->rotation();
				primitive.shape = body->shape();
				primitive.transform.position = body->position();
				if (Detector::collide(primitive, sensorRegion))
					body->forces() += (sensorRegion.transform.position - body->position()).normal() * force;
			}
		}

	private:
		Rectangle rectangle;
		Edge edge;
		Circle circle;
		ShapePrimitive sensorRegion;

		real force = 250.0f;
	};
}
#endif
