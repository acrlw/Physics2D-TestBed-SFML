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

			body1 = m_settings.world->createBody();
			body1->setShape(&rect);
			body1->position().set({ 0.0f, 0.0f });
			body1->setMass(1.0f);
			body1->setType(Body::BodyType::Dynamic);
			body1->setFriction(0.5f);
			body1->setRestitution(0.0f);

			m_settings.tree->insert(body1);

			body2 = m_settings.world->createBody();
			body2->setShape(&rect);
			body2->position().set({ 2.0f, -2.0f });
			body2->setMass(1.0f);
			body2->setType(Body::BodyType::Dynamic);
			body2->setFriction(0.5f);
			body2->setRestitution(0.0f);
			body2->rotation() = Math::degreeToRadian(30.0f);

			m_settings.tree->insert(body2);

			body3 = m_settings.world->createBody();
			body3->setShape(&rect);
			body3->position().set({ 4.0f, 2.0f });
			body3->setMass(1.0f);
			body3->setType(Body::BodyType::Dynamic);
			body3->setFriction(0.5f);
			body3->setRestitution(0.0f);
			body3->rotation() = Math::degreeToRadian(-30.0f);

			m_settings.tree->insert(body3);

			m_settings.world->setEnableGravity(false);
			
		}

		void onPostRender(sf::RenderWindow& window) override
		{

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
		void onRenderUI() override
		{
			Vector2 pos(6.0f, -3.0f);
			pos = m_settings.camera->worldToScreen(pos);
			ImGui::SetNextWindowPos(ImVec2(pos.x, pos.y), ImGuiCond_Once);

			ImGui::Begin("Solver Helper");
			ImGui::Text("Press S to start position solver.");
			ImGui::End();
		}
		void onUnLoad() override
		{
			m_settings.world->setEnableGravity(true);
		}
	private:
		Rectangle rect;
		Edge edge;
		Body* body1, *body2, *body3;
	};
}
#endif
