#ifndef FRAME_H
#define FRAME_H
#include "physics2d.h"
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Graphics.hpp>

#include "camera.h"
#include "render.h"

namespace Physics2D
{
	struct FrameSettings
	{
		//Physics
		PhysicsWorld* world = nullptr;
		ContactMaintainer* maintainer = nullptr;
		Tree* tree = nullptr;
		UniformGrid* grid = nullptr;
		PhysicsSystem* system = nullptr;

		//Render
		Camera* camera = nullptr;
		sf::Font* font = nullptr;
	};
	class Frame
	{
	public:
		Frame(const FrameSettings& settings) : m_settings(settings)
		{
		}

		virtual ~Frame()
		{
		}

		virtual void onPostStep(real dt)
		{
		}

		virtual void onPreStep(real dt)
		{
		}

		virtual void onLoad()
		{
		}

		virtual void onUnLoad()
		{
		}

		virtual void onPreRender(sf::RenderWindow& window)
		{
		}

		virtual void onPostRender(sf::RenderWindow& window)
		{
		}

		virtual void onRenderUI()
		{
		}

		virtual void onMousePress(sf::Event& event)
		{
		}

		virtual void onMouseRelease(sf::Event& event)
		{
		}

		virtual void onMouseMove(sf::Event& event)
		{
		}

		virtual void onMouseDoubleClick(sf::Event& event)
		{
		}

		virtual void onKeyRelease(sf::Event& event)
		{
		}

		virtual void onKeyPressed(sf::Event& event)
		{
		}

		void setCurrentBody(Body* body) { m_currentBody = body; }
		Body* currentBody() const { return m_currentBody; }


	protected:
		FrameSettings m_settings;
		Body* m_currentBody = nullptr;
	};
}
#endif
