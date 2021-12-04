#ifndef TESTBED_H
#define TESTBED_H
#include "imgui.h"
#include "imgui-SFML.h"
#include <iostream>
#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Graphics.hpp>
namespace Physics2D
{
	class TestBed
	{
	public:
		void exec();
	private:
		void render(sf::RenderWindow& window);
		void renderGUI(sf::RenderWindow& window, sf::Clock& clock);

		void pause();
		void restart();
		void step();

		int m_positionIteration = 8;
		int m_velocityIteration = 6;
		int m_frequency = 120;
		float m_contactBiasFactor = 0.03f;

		bool m_bodyVisible = true;
		bool m_aabbVisible = false;
		bool m_jointVisible = true;
		bool m_gridVisible = false;
		bool m_treeVisible = false;
		bool m_contactsVisible = false;
		bool m_axisVisible = false;
		bool m_userDrawVisible = true;
		bool m_angleVisible = false;
		bool m_centerVisible = false;

		
		int m_currentItem = 0;
	};
}
#endif