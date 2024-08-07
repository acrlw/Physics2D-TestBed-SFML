#ifndef PHYSICS2D_SCENES_PBD_H
#define PHYSICS2D_SCENES_PBD_H
#include "frame.h"

namespace Physics2D
{
	class PBDFrame : public Frame
	{
	public:
		PBDFrame(const FrameSettings& settings) : Frame(settings)
		{
		}

		void onLoad() override
		{
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_real_distribution<> pos(-50.0f, 50.0f);
			for (int i = 0; i < 10; ++i)
			{
				positions[i].set(pos(gen), pos(gen));
				prePositions[i] = positions[i];
				invMasses[i] = 0.5;
			}
		}

		void onPostRender(sf::RenderWindow& window) override
		{
			sf::Color color = sf::Color::Yellow;
			for (int i = 0; i < 10; ++i)
				RenderSFMLImpl::renderPoint(window, *m_settings.camera, positions[i], color);

			for (int i = 0; i < 9; ++i)
			{
				color.a = 150;
				RenderSFMLImpl::renderLine(window, *m_settings.camera, positions[i], positions[i + 1], color);
			}
		}

		void onPostStep(real dt) override
		{
			for (size_t i = 0; i < 10; ++i)
			{
				velocities[i] += dt * gravity;
				prePositions[i] = positions[i];
				positions[i] += dt * velocities[i];
			}
			for (size_t j = 0; j < m_settings.system->positionIteration(); ++j)
			{
				for (size_t i = 0; i < 9; ++i)
				{
					real k = invMasses[i] / (invMasses[i] + invMasses[i + 1]);
					Vector2 l = (positions[i + 1] - positions[i]);
					real error = l.length() - distance;
					Vector2 deltaX1 = k * error * l.normal();
					Vector2 deltaX2 = -k * error * l.normal();
					positions[i] += deltaX1;
					positions[i + 1] += deltaX2;
				}
			}
			//update velocities
			for (size_t i = 0; i < 10; ++i)
			{
				velocities[i] = (positions[i] - prePositions[i]) / dt;
			}
			positions[0].clear();
			velocities[0].clear();
			positions[9].set(18.0f, 0.0f);
			velocities[9].clear();
		}

	private:
		Vector2 positions[10];
		Vector2 prePositions[10];
		Vector2 velocities[10];
		real invMasses[10];
		real distance = 2;
		Vector2 gravity = {0.0, -9.8f};
		uint32_t iterations = 10;
	};
}
#endif
