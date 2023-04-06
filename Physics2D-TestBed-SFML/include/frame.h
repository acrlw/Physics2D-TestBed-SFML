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
	class Frame
	{
	public:
		Frame(std::string name, PhysicsWorld* world, ContactMaintainer* maintainer,
		      Tree* tree, UniformGrid* grid, Camera* camera) : m_name(name), m_world(world), m_maintainer(maintainer),
		                                                       m_tree(tree), m_grid(grid), m_camera(camera)
		{
		}

		virtual ~Frame()
		{
		}

		virtual void postStep(real dt)
		{
		}

		virtual void preStep(real dt)
		{
		}

		virtual void load()
		{
		}

		virtual void release()
		{
		}

		virtual void preRender(sf::RenderWindow& window)
		{
		}

		virtual void render(sf::RenderWindow& window)
		{
		}

		virtual void renderUI()
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

		std::string name() const
		{
			return m_name;
		}

		void setName(const std::string& name)
		{
			m_name = name;
		}

	protected:
		std::string m_name;
		PhysicsWorld* m_world = nullptr;
		ContactMaintainer* m_maintainer = nullptr;
		Tree* m_tree = nullptr;
		UniformGrid* m_grid = nullptr;
		Camera* m_camera = nullptr;
		Body* m_currentBody = nullptr;
	};
}
#endif
