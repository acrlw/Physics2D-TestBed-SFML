#ifndef PHYSICS2D_SCENES_PBD_H
#define PHYSICS2D_SCENES_PBD_H
#include "./include/frame.h"
namespace Physics2D
{
	class PBDFrame : public Frame
	{
	public:
		PBDFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
			Tree* tree, DBVH* dbvh, Camera* camera) : Frame("Position-Based Dynamics", world, maintainer, tree, dbvh, camera)
		{

		}
		void load() override
		{
			for(int i = 0;i < 10; ++i)
			{
				positions[i].x = static_cast<real>(i) * 2.0;
				prePositions[i].x = static_cast<real>(i) * 2.0;
				invMasses[i] = 0.5;
			}
		}
		void render(sf::RenderWindow& window) override
		{
			sf::Color color = sf::Color::Yellow;
			for (int i = 0; i < 10; ++i)
				RenderSFMLImpl::renderPoint(window, *m_camera, positions[i], color);

			for (int i = 0; i < 9; ++i)
			{
				color.a = 150;
				RenderSFMLImpl::renderLine(window, *m_camera, positions[i], positions[i + 1], color);
			}
		}
		void update(real dt) override
		{
			for(size_t i = 0;i < 10;++i)
			{
				velocities[i] += dt * gravity;
				prePositions[i] = positions[i];
				positions[i] += dt * velocities[i];
			}
			for(size_t j = 0;j < iterations;++j)
			{
				for(size_t i = 0;i < 9; ++i)
				{
					real k = invMasses[i] / (invMasses[i] + invMasses[i + 1]);
					Vec2 l = (positions[i + 1] - positions[i]);
					real error = l.magnitude() - distance;
					Vec2 deltaX1 = k * error * l.normal();
					Vec2 deltaX2 = -k * error * l.normal();
					positions[i] += deltaX1;
					positions[i + 1] += deltaX2;
				}
			}
			//update velocities
			for(size_t i = 0;i < 10;++i)
			{
				velocities[i] = (positions[i] - prePositions[i]) / dt;
			}
			positions[0].clear();
			velocities[0].clear();
			positions[9] = prePositions[9];
			velocities[9].clear();
		}
	private:
		Vec2 positions[10];
		Vec2 prePositions[10];
		Vec2 velocities[10];
		real invMasses[10];
		real distance = 2;
		Vec2 gravity = {0.0, -9.8f};
		uint32_t iterations = 10;
	};
}
#endif