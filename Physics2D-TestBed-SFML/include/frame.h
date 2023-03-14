#ifndef FRAME_H
#define FRAME_H
#include "../dependencies/Physics2D/physics2d.h"
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
			Tree* tree, DBVH* dbvh, Camera* camera) : m_name(name), m_world(world), m_maintainer(maintainer),
			m_tree(tree), m_dbvh(dbvh), m_camera(camera) {}
		virtual void update(real dt) {}
		virtual void load() {}
		virtual void release() {}
		virtual void render(sf::RenderWindow& window) {}
		virtual void renderUI() {}
		virtual void onMousePress(sf::Event& event) {}
		virtual void onMouseRelease(sf::Event& event) {}
		virtual void onMouseMove(sf::Event& event) {}
		virtual void onMouseDoubleClick(sf::Event& event) {}
		virtual void onKeyRelease(sf::Event& event) {}
		virtual void onKeyPressed(sf::Event& event) {}

		std::string name()const
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
		DBVH* m_dbvh = nullptr;
		Camera* m_camera = nullptr;
	};
}
#endif