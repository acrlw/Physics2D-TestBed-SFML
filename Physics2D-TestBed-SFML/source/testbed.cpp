#include "testbed.h"

namespace Physics2D
{
	TestBed::TestBed()
	{
		m_frameList = {
			{
				[&]
				{
					return new BitmaskFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(),
					                        &m_system.grid(), &m_camera);
				},
				[&]
				{
					return new BridgeFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(),
					                       &m_system.grid(), &m_camera);
				},
				[&]
				{
					return new BroadPhaseFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(),
					                           &m_system.grid(), &m_camera);
				},
				[&]
				{
					return new ChainFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(), &m_system.grid(),
					                      &m_camera);
				},
				[&]
				{
					return new CollisionFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(),
					                          &m_system.grid(), &m_camera);
				},
				[&]
				{
					return new ContinuousFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(),
					                           &m_system.grid(), &m_camera);
				},
				[&]
				{
					return new CustomFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(),
					                       &m_system.grid(), &m_camera);
				},
				[&]
				{
					return new DominoFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(),
					                       &m_system.grid(), &m_camera);
				},
				[&]
				{
					return new FrictionFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(),
					                         &m_system.grid(), &m_camera);
				},
				[&]
				{
					return new GeometryFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(),
					                         &m_system.grid(), &m_camera);
				},
				[&]
				{
					return new JointsFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(),
					                       &m_system.grid(), &m_camera);
				},
				[&]
				{
					return new NarrowphaseFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(),
					                            &m_system.grid(), &m_camera);
				},
				[&]
				{
					return new NewtonCradleFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(),
					                             &m_system.grid(), &m_camera);
				},
				[&]
				{
					return new PBDFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(), &m_system.grid(),
					                    &m_camera);
				},
				[&]
				{
					return new PendulumFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(),
					                         &m_system.grid(), &m_camera);
				},
				[&]
				{
					return new RaycastFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(),
					                        &m_system.grid(), &m_camera);
				},
				[&]
				{
					return new RestitutionFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(),
					                            &m_system.grid(), &m_camera);
				},
				[&]
				{
					return new SensorFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(),
					                       &m_system.grid(), &m_camera);
				},
				[&]
				{
					return new StackingFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(),
					                         &m_system.grid(), &m_camera);
				},
				[&]
				{
					return new WreckingBallFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(),
					                             &m_system.grid(), &m_camera);
				},
				[&]
				{
					return new XPBDFrame(&m_system.world(), &m_system.maintainer(), &m_system.tree(), &m_system.grid(),
					                     &m_camera);
				}
			}
		};

		m_system.world().setEnableGravity(true);
		m_system.world().setLinearVelocityDamping(0.1f);
		m_system.world().setAirFrictionCoefficient(0.8f);
		m_system.world().setAngularVelocityDamping(0.1f);
		m_system.world().setEnableDamping(true);
		m_system.positionIteration() = 3;
		m_system.velocityIteration() = 6;

		m_pointJointPrimitive.bodyA = nullptr;
		m_mouseJoint = m_system.world().createJoint(m_pointJointPrimitive);
		m_mouseJoint->setActive(false);

		m_camera.setViewport(Camera::Viewport(Vector2(0, 0), Vector2(1920, 1080)));
		m_camera.setWorld(&m_system.world());

		m_camera.setTree(&m_system.tree());
		m_camera.setContactMaintainer(&m_system.maintainer());
		m_camera.setUniformGrid(&m_system.grid());

		changeFrame();
	}

	TestBed::~TestBed()
	{
	}

	void TestBed::onResized(sf::Event& event)
	{
		Camera::Viewport viewport = m_camera.viewport();
		viewport.set(static_cast<real>(event.size.width), static_cast<real>(event.size.height));
		m_camera.setViewport(viewport);
		m_window->setView(sf::View(sf::FloatRect(0, 0, event.size.width, event.size.height)));
	}

	void TestBed::onClosed(sf::Event& event)
	{
		m_window->close();
	}

	void TestBed::onKeyReleased(sf::Event& event)
	{
		switch (event.key.code)
		{
		case sf::Keyboard::Space:
			{
				m_running = !m_running;
				break;
			}
		case sf::Keyboard::N:
			{
				//N means next
				step();
				break;
			}
		case sf::Keyboard::T:
			{
				//T means stepping twice
				step();
				step();
				break;
			}
		case sf::Keyboard::R:
			{
				restart();
				break;
			}
		case sf::Keyboard::LAlt:
			{
				m_onDistanceCheck = false;
				m_mouseArray[0].clear();
				m_mouseArray[1].clear();
				break;
			}
		default:
			break;
		}
		if (m_currentFrame != nullptr)
			m_currentFrame->onKeyRelease(event);
	}

	void TestBed::onKeyPressed(sf::Event& event)
	{
		if (m_currentFrame != nullptr)
			m_currentFrame->onKeyPressed(event);

		if (event.key.code == sf::Keyboard::LAlt)
			m_onDistanceCheck = true;
	}

	void TestBed::onMouseReleased(sf::Event& event)
	{
		Vector2 pos(static_cast<real>(event.mouseButton.x), static_cast<real>(event.mouseButton.y));
		m_mousePos = m_camera.screenToWorld(pos);
		m_screenMousePos = pos;

		if (m_currentFrame != nullptr)
			m_currentFrame->onMouseRelease(event);

		if (m_mouseJoint == nullptr)
			return;
		m_mouseJoint->primitive().clear();
		m_mouseJoint->setActive(false);

		m_cameraViewportMovement = false;
		m_selectedBody = nullptr;
		m_currentFrame->setCurrentBody(nullptr);
	}

	void TestBed::onMouseMoved(sf::Event& event)
	{
		if (m_currentFrame != nullptr)
			m_currentFrame->onMouseMove(event);

		Vector2 pos(static_cast<real>(event.mouseMove.x), static_cast<real>(event.mouseMove.y));
		m_screenMousePos = pos;

		Vector2 tf = m_camera.screenToWorld(pos) - m_mousePos;
		if (m_cameraViewportMovement)
		{
			tf *= m_camera.meterToPixel();
			m_camera.setTransform(m_camera.transform() + tf);
		}
		m_mousePos = m_camera.screenToWorld(pos);

		if (m_onDistanceCheck)
		{
			if (m_mouseArray[0].isOrigin())
				m_mouseArray[0] = m_mousePos;
			else
				m_mouseArray[1] = m_mousePos;
		}

		if (m_currentFrame != nullptr)
			m_currentFrame->onMouseMove(event);

		if (m_mouseJoint == nullptr)
			return;

		auto prim = m_mouseJoint->primitive();
		prim.targetPoint = m_mousePos;

		m_mouseJoint->set(prim);
	}

	void TestBed::onMousePressed(sf::Event& event)
	{
		Vector2 pos(static_cast<real>(event.mouseButton.x), static_cast<real>(event.mouseButton.y));
		m_screenMousePos = pos;
		m_mousePos = m_camera.screenToWorld(pos);

		if (event.mouseButton.button == sf::Mouse::Right)
			m_cameraViewportMovement = true;

		if (m_currentFrame != nullptr)
			m_currentFrame->onMousePress(event);

		if (event.mouseButton.button == sf::Mouse::Left && m_mouseJoint != nullptr)
		{
			AABB mouseBox;
			mouseBox.position = m_mousePos;
			mouseBox.width = 0.01f;
			mouseBox.height = 0.01f;
			auto bodies = m_system.tree().query(mouseBox);
			//auto bodies = m_system.grid().query(mouseBox);
			for (auto& body : bodies)
			{
				Vector2 point = m_mousePos - body->position();
				point = Matrix2x2(-body->rotation()).multiply(point);
				if (body->shape()->contains(point) && m_selectedBody == nullptr && body->type() !=
					Body::BodyType::Static)
				{
					m_selectedBody = body;
					PointJointPrimitive prim;
					prim.localPointA = body->toLocalPoint(m_mousePos);
					prim.bodyA = body;
					prim.targetPoint = m_mousePos;
					prim.maxForce = 1000 * body->mass();
					m_mouseJoint->set(prim);
					m_mouseJoint->prepare(static_cast<real>(1 / m_frequency));

					m_mouseJoint->setActive(true);
					m_selectedBody->setSleep(false);
					m_currentFrame->setCurrentBody(m_selectedBody);
					break;
				}
			}
		}
	}

	void TestBed::onWheelScrolled(sf::Event& event)
	{
		m_camera.setPreScreenMousePos(m_screenMousePos);
		if (event.mouseWheelScroll.delta > 0)
			m_camera.setTargetMeterToPixel(m_camera.meterToPixel() + m_camera.meterToPixel() / 2.0f);
		else
			m_camera.setTargetMeterToPixel(m_camera.meterToPixel() - m_camera.meterToPixel() / 2.0f);
		
	}

	void TestBed::exec()
	{
		// create the window
		sf::ContextSettings settings;
		settings.antialiasingLevel = 8;
		m_window = std::make_unique<sf::RenderWindow>(sf::VideoMode(1920, 1080), "Testbed", sf::Style::Default,
		                                              settings);
		ImGui::SFML::Init(*m_window);
		m_window->setActive(false);
		m_window->setFramerateLimit(60);

		auto& io = ImGui::GetIO();
		io.Fonts->AddFontFromFileTTF("font/MiSans-Medium.ttf", 18.0f);
		io.FontDefault = io.Fonts->Fonts[1];
		ImGui::SFML::UpdateFontTexture();

		sf::Clock deltaClock;
		while (m_window->isOpen())
		{
			sf::Event event{};
			while (m_window->pollEvent(event))
			{
				ImGui::SFML::ProcessEvent(event);

				switch (event.type)
				{
				case sf::Event::Closed:
					{
						onClosed(event);
						break;
					}
				case sf::Event::KeyReleased:
					{
						onKeyReleased(event);
						break;
					}
				case sf::Event::MouseButtonPressed:
					{
						onMousePressed(event);
						break;
					}
				case sf::Event::MouseButtonReleased:
					{
						onMouseReleased(event);
						break;
					}
				case sf::Event::MouseMoved:
					{
						onMouseMoved(event);
						break;
					}
				case sf::Event::MouseWheelScrolled:
					{
						onWheelScrolled(event);
						break;
					}
				case sf::Event::Resized:
					{
						onResized(event);
						break;
					}
				case sf::Event::KeyPressed:
					{
						onKeyPressed(event);
						break;
					}
				default:
					break;
				}
			}

			const bool show = m_currentFrame != nullptr && m_userDrawVisible;
			if (show)
				m_currentFrame->preRender(*m_window);

			simulate();

			m_camera.render(*m_window);
			if (show)
				m_currentFrame->render(*m_window);

			renderGUI(*m_window, deltaClock);

			render(*m_window);

			m_window->display();
		}
		ImGui::SFML::Shutdown();
	}

	void TestBed::renderGUI(sf::RenderWindow& window, sf::Clock& clock)
	{
		const char* items[] = {
			"Bitmask", "Bridge", "Broadphase", "Chain", "Collision", "Continuous", "Custom", "Domino", "Friction",
			"Geometry", "Joints", "Narrowphase", "Newton's Cradle", "Position-Based Dynamics", "Pendulum",
			"AABB Raycast", "Restitution", "Sensor", "Stacking",
			"Wrecking Ball", "Extended Position-Based Dynamics"
		};


		ImGui::SFML::Update(window, clock.restart());
		ImGui::SetWindowPos("Panel", ImVec2(0, 0));

		ImGui::Begin("Panel", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

		ImGuiStyle& style = ImGui::GetStyle();
		style.WindowRounding = 5.0f;
		style.Colors[ImGuiCol_WindowBg] = ImVec4(0.1f, 0.1f, 0.1f, 0.55f);


		ImGui::Text("Scenes");

		int oldItem = m_currentItem;
		ImGui::Combo("Current Scene", &m_currentItem, items, IM_ARRAYSIZE(items));
		if (oldItem != m_currentItem)
			change();


		ImGui::Separator();
		ImGui::Text("Iteration");
		ImGui::SliderInt("Position Iteration", &m_system.positionIteration(), 1, 20);
		ImGui::SliderInt("Velocity Iteration", &m_system.velocityIteration(), 1, 20);

		ImGui::Separator();
		ImGui::Text("Time");

		ImGui::SliderInt("Delta Time", &m_frequency, 30, 240);
		ImGui::Checkbox("Slice Delta Time", &m_system.sliceDeltaTime());

		ImGui::Separator();
		ImGui::Text("Penetration");
		ImGui::SliderFloat("Bias Factor", &m_system.maintainer().m_biasFactor, 0.01f, 0.50f, "%.2f");
		ImGui::SliderFloat("Max Penetration", &m_system.maintainer().m_maxPenetration, 0.001f, 0.1f);

		ImGui::Separator();
		ImGui::Text("Body");
		ImGui::Checkbox("Sleep", &m_system.world().enableSleep());

		ImGui::Separator();
		ImGui::Text("Solver");
		ImGui::Columns(2, nullptr);
		ImGui::Checkbox("Solve Joint Vel", &m_system.solveJointVelocity());
		ImGui::Checkbox("Solve Joint Pos", &m_system.solveJointPosition());
		ImGui::NextColumn();
		ImGui::Checkbox("Solve Contact Vel", &m_system.solveContactVelocity());
		ImGui::Checkbox("Solve Contact Pos", &m_system.solveContactPosition());
		ImGui::NextColumn();
		ImGui::Checkbox("Warmstart", &m_system.maintainer().m_warmStart);
		ImGui::NextColumn();
		ImGui::Columns(1, nullptr);

		ImGui::Separator();
		ImGui::Text("Render");
		ImGui::Columns(2, nullptr);
		ImGui::Checkbox("Body", &m_camera.bodyVisible());
		ImGui::Checkbox("Joint", &m_camera.jointVisible());
		ImGui::Checkbox("Center", &m_camera.centerVisible());
		ImGui::Checkbox("User Draw", &m_userDrawVisible);
		ImGui::NextColumn();

		ImGui::Checkbox("AABB", &m_camera.aabbVisible());
		ImGui::Checkbox("Grid Scale Line", &m_camera.gridScaleLineVisible());
		ImGui::Checkbox("Tree", &m_camera.treeVisible());
		ImGui::Checkbox("Uniform Grid", &m_camera.uniformGridVisible());
		ImGui::NextColumn();
		ImGui::Columns(1, nullptr);

		ImGui::Separator();
		ImGui::Text("Data");
		ImGui::Columns(2, nullptr);
		ImGui::Checkbox("Contacts", &m_camera.contactVisible());
		ImGui::Checkbox("Contacts Impulse", &m_camera.contactImpulseVisible());
		ImGui::Checkbox("Contacts Impulse Mag", &m_camera.contactImpulseMagnitude());
		ImGui::Checkbox("Contacts Friction", &m_camera.contactFrictionVisible());
		ImGui::NextColumn();

		ImGui::Checkbox("Contacts Friction Mag", &m_camera.contactFrictionMagnitude());
		ImGui::Checkbox("Linear Velocity", &m_camera.bodyVelocity());
		ImGui::Checkbox("Linear Velocity Mag", &m_camera.bodyVelocityMagnitude());
		ImGui::Checkbox("Linear Velocity Normal", &m_camera.bodyVelocityNormal());
		ImGui::NextColumn();

		ImGui::Columns(1, nullptr);

		ImGui::Separator();
		ImGui::Text("Running: %s", m_running ? "True" : "False");

		ImGui::Separator();
		ImGui::Text("Button");
		if (ImGui::Button("Pause", ImVec2(-FLT_MIN, 0.0f)))
			pause();
		if (ImGui::Button("Step", ImVec2(-FLT_MIN, 0.0f)))
			step();
		if (ImGui::Button("Restart", ImVec2(-FLT_MIN, 0.0f)))
			restart();

		ImGui::End();

		if (m_currentFrame != nullptr)
			m_currentFrame->renderUI();

		ImGui::SFML::Render(window);
	}

	void TestBed::render(sf::RenderWindow& window)
	{
		if (m_onDistanceCheck)
		{
			if (m_mouseArray[0].isOrigin() || m_mouseArray[1].isOrigin())
				return;
			Vector2 v = m_mouseArray[1] - m_mouseArray[0];
			real length = v.length();
			if (realEqual(length, 0))
				return;
			Vector2 normal = v / length;
			RenderSFMLImpl::renderPoint(window, m_camera, m_mouseArray[0], RenderConstant::Green);
			RenderSFMLImpl::renderPoint(window, m_camera, m_mouseArray[1], RenderConstant::Green);
			RenderSFMLImpl::renderLine(window, m_camera, m_mouseArray[0], m_mouseArray[1], RenderConstant::Green);
			std::string str = std::format("{:.6f}", length);
			sf::Text text;
			text.setFont(m_camera.font());
			text.setString(str);
			text.setCharacterSize(16);
			text.setFillColor(RenderConstant::Green);
			sf::FloatRect text_rect = text.getLocalBounds();
			text.setOrigin(text_rect.left + text_rect.width / 2.0f, text_rect.top + text_rect.height / 2.0f);
			const Vector2 t = normal.perpendicular();
			Vector2 half(text_rect.width / 2.0f, text_rect.height / 2.0f);
			half *= m_camera.pixelToMeter();
			Vector2 offset = t * half.x * 1.2f - normal * half.y * 1.5f;
			text.setPosition(RenderSFMLImpl::toVector2f(m_camera.worldToScreen(m_mouseArray[1] + offset)));
			text.rotate(Math::radianToDegree(-t.theta()));
			window.draw(text);
		}
	}

	void TestBed::pause()
	{
		m_running = !m_running;
	}

	void TestBed::restart()
	{
		change();
	}

	void TestBed::step()
	{
		const real dt = 1.0f / static_cast<real>(m_frequency);
		const bool valid = m_currentFrame != nullptr;
		if (valid)
			m_currentFrame->preStep(dt);

		m_system.step(dt);

		if (valid)
			m_currentFrame->postStep(dt);
	}

	void TestBed::simulate()
	{
		if (m_running)
			step();
	}

	void TestBed::change()
	{
		changeFrame();
	}

	void TestBed::changeFrame()
	{
		clearAll();
		//optimize this code
		m_currentFrame = m_frameList[m_currentItem]();
		if (m_currentFrame != nullptr)
			m_currentFrame->load();
	}

	void TestBed::clearAll()
	{
		m_system.world().clearAllBodies();
		m_system.world().clearAllJoints();
		m_system.maintainer().clearAll();
		m_system.tree().clearAll();
		m_system.grid().clearAll();
		m_pointJointPrimitive.bodyA = nullptr;
		m_mouseJoint = m_system.world().createJoint(m_pointJointPrimitive);
		m_mouseJoint->setActive(false);
		if (m_currentFrame != nullptr)
		{
			m_currentFrame->release();
			delete m_currentFrame;
			m_currentFrame = nullptr;
		}
	}
}
