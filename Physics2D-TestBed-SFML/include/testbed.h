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
		void renderGUI(sf::RenderWindow& window, sf::Clock& clock);

		void pause();
		void restart();
		void step();

		void simulate();

		void change();
		void changeFrame();
		void clearAll();

		int m_positionIteration = 8;
		int m_velocityIteration = 6;
		int m_frequency = 120;
		float m_contactBiasFactor = 0.03f;

		bool m_userDrawVisible = true;
		bool m_running = true;
		bool m_cameraViewportMovement = false;

		
		int m_currentItem = 0;

		Frame* m_currentFrame = nullptr;

		PhysicsWorld m_world;
		ContactMaintainer m_maintainer;
		Tree m_tree;
		DBVH m_dbvh;
		Body* m_selectedBody;
		PointJoint* m_mouseJoint;
		PointJointPrimitive m_pointJointPrimitive;
		Camera m_camera;

		Vector2 m_mousePos;

		std::unique_ptr<sf::Thread> m_physicsThread;
		std::mutex m_mutexChangeFrame;
		std::mutex m_mutexSimulate;
		std::condition_variable m_cvChangeFrame;
		std::condition_variable m_cvSimulate;

		bool m_simulateWorkingState = true;
	};
}
#endif