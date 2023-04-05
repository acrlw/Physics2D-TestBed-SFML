#ifndef TESTBED_H
#define TESTBED_H
#include "imgui.h"
#include "imgui-SFML.h"

#include <iostream>

#include "scenes/bitmask.h"
#include "scenes/bridge.h"
#include "scenes/broadphase.h"
#include "scenes/chain.h"
#include "scenes/collision.h"
#include "scenes/continuous.h"
#include "scenes/custom.h"
#include "scenes/domino.h"
#include "scenes/friction.h"
#include "scenes/geometry.h"
#include "scenes/joints.h"
#include "scenes/narrowphase.h"
#include "scenes/newtoncradle.h"
#include "scenes/raycast.h"
#include "scenes/restitution.h"
#include "scenes/sensor.h"
#include "scenes/stacking.h"
#include "scenes/wreckingball.h"
#include "scenes/pendulum.h"
#include "scenes/pbd.h"
#include "scenes/xpbd.h"

#include <mutex>
#include <thread>
#include <condition_variable>
#include <chrono>
namespace Physics2D
{
	class TestBed
	{
	public:
		TestBed();
		~TestBed();
		void exec();
	private:
        //events
        void onResized(sf::Event &event);
        void onClosed(sf::Event &event);
        void onKeyReleased(sf::Event &event);
		void onKeyPressed(sf::Event& event);
        void onMouseReleased(sf::Event &event);
        void onMouseMoved(sf::Event &event);
        void onMousePressed(sf::Event &event);
        void onWheelScrolled(sf::Event& event);
        //render
		void renderGUI(sf::RenderWindow& window, sf::Clock& clock);
		void render(sf::RenderWindow& window);
        //simulation
		void pause();
		void restart();
		void step();

		void simulate();

		void change();
		void changeFrame();
		void clearAll();

		int m_frequency = 60;

		bool m_userDrawVisible = true;
		bool m_running = true;
		bool m_cameraViewportMovement = false;

		
		int m_currentItem = 6;

		Frame* m_currentFrame = nullptr;

		PhysicsSystem m_system;
		
		Body* m_selectedBody = nullptr;
		PointJoint* m_mouseJoint = nullptr;
		PointJointPrimitive m_pointJointPrimitive;
		Camera m_camera;

		Vector2 m_mousePos;
		

		bool m_simulateWorkingState = true;
        std::unique_ptr<sf::RenderWindow> m_window;

		std::array<std::function<Frame* ()>, 21> m_frameList;

		bool m_onDistanceCheck = false;
		Vector2 m_mouseArray[2];
    };
}
#endif