#ifndef PHYSICS2D_SCENES_SOLVER_H
#define PHYSICS2D_SCENES_SOLVER_H
#include "frame.h"

namespace Physics2D
{
	class SolverFrame : public Frame
	{
	public:
		SolverFrame(const FrameSettings& settings) : Frame(settings)
		{
		}

		void onLoad() override
		{
			rect.set(1.0f, 1.0f);
			body1.position.set(0.5, 0.4f);
			edge.set(-10.0f, 10.0f);
		}

		void onPostRender(sf::RenderWindow& window) override
		{
			ShapePrimitive sp;
			sp.shape = &rect;
			sp.transform = body1;
			RenderSFMLImpl::renderRectangle(window, *m_settings.camera, sp, RenderConstant::Green);
			sp.shape = &edge;
			sp.transform.position.clear();
			RenderSFMLImpl::renderEdge(window, *m_settings.camera, sp, RenderConstant::Cyan);
		}

		void onPostStep(real dt) override
		{
			
		}

		void onKeyPressed(sf::Event& event) override
		{
			if (event.key.code == sf::Keyboard::S)
			{
				//Solver runs

			}
		}
	private:
		Rectangle rect;
		Edge edge;
		Transform body1;

	};
}
#endif
