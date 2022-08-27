#include "..\include\testbed.h"
namespace Physics2D
{
    TestBed::TestBed()
    {

        m_system.world().setEnableGravity(true);
	    m_system.world().setLinearVelocityDamping(0.1f);
	    m_system.world().setAirFrictionCoefficient(0.8f);
	    m_system.world().setAngularVelocityDamping(0.1f);
	    m_system.world().setEnableDamping(true);
		m_system.positionIteration() = 6;
		m_system.velocityIteration() = 8;

        m_pointJointPrimitive.bodyA = nullptr;
        m_mouseJoint = m_system.world().createJoint(m_pointJointPrimitive);
        m_mouseJoint->setActive(false);

        m_camera.setViewport(Camera::Viewport(Vec2(0, 0), Vec2(1920, 1080)));
        m_camera.setWorld(&m_system.world());
        m_camera.setDbvh(&m_dbvh);
        m_camera.setTree(&m_system.tree());
        m_camera.setContactMaintainer(&m_system.maintainer());

        m_physicsThread = std::make_unique<sf::Thread>(&TestBed::simulate, this);
        changeFrame();


    }
    TestBed::~TestBed()
    {

    }
    void TestBed::onResized(sf::Event& event)
    {
        Camera::Viewport viewport = m_camera.viewport();
        viewport.set(real(event.size.width), real(event.size.height));
        m_camera.setViewport(viewport);
        m_window->setView(sf::View(sf::FloatRect(0, 0, event.size.width, event.size.height)));
    }
    void TestBed::onClosed(sf::Event& event)
    {
        m_simulateWorkingState = false;
        m_physicsThread->wait();
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
    }
    void TestBed::onMouseReleased(sf::Event& event)
    {
        Vec2 pos(real(event.mouseButton.x), real(event.mouseButton.y));
        m_mousePos = m_camera.screenToWorld(pos);

        if (m_currentFrame != nullptr)
            m_currentFrame->onMouseRelease(event);

        if (m_mouseJoint == nullptr)
            return;
        m_mouseJoint->setActive(false);

        m_cameraViewportMovement = false;
        m_selectedBody = nullptr;
    }
    void TestBed::onMouseMoved(sf::Event& event)
    {
        if (m_currentFrame != nullptr)
            m_currentFrame->onMouseMove(event);

        Vec2 pos(real(event.mouseMove.x), real(event.mouseMove.y));

        Vec2 tf = m_camera.screenToWorld(pos) - m_mousePos;
        if (m_cameraViewportMovement)
        {
            tf *= m_camera.meterToPixel();
            m_camera.setTransform(m_camera.transform() + tf);
        }
        m_mousePos = m_camera.screenToWorld(pos);

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
        Vec2 pos(real(event.mouseButton.x), real(event.mouseButton.y));
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
            for (auto& body : bodies)
            {
                Vec2 point = m_mousePos - body->position();
                point = Mat2(-body->rotation()).multiply(point);
                if (body->shape()->contains(point) && m_selectedBody == nullptr && body->type() != BodyType::Static)
                {
                    m_selectedBody = body;
                    auto prim = m_mouseJoint->primitive();
                    prim.localPointA = body->toLocalPoint(m_mousePos);
                    prim.bodyA = body;
                    prim.targetPoint = m_mousePos;
                    m_mouseJoint->setActive(true);
                    m_mouseJoint->set(prim);
                    m_selectedBody->setSleep(false);
                    break;
                }
            }
        }
    }

    void TestBed::onWheelScrolled(sf::Event &event) {
        if (event.mouseWheelScroll.delta > 0)
            m_camera.setMeterToPixel(m_camera.meterToPixel() + m_camera.meterToPixel() / 4.0f);
        else
            m_camera.setMeterToPixel(m_camera.meterToPixel() - m_camera.meterToPixel() / 4.0f);
    }
    void TestBed::exec()
	{
        // create the window
        sf::ContextSettings settings;
        settings.antialiasingLevel = 16;
        m_window = std::make_unique<sf::RenderWindow>(sf::VideoMode(1920, 1080), "Testbed", sf::Style::Default, settings);
        ImGui::SFML::Init(*m_window);
        m_window->setActive(false);
        m_window->setFramerateLimit(120);
        m_physicsThread->launch();

        sf::Clock deltaClock;
        while (m_window->isOpen())
        {
            sf::Event event{};
            while (m_window->pollEvent(event)) {
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

            m_camera.render(*m_window);
            if (m_currentFrame != nullptr && m_userDrawVisible)
                m_currentFrame->render(*m_window);

            renderGUI(*m_window, deltaClock);

        }
        ImGui::SFML::Shutdown();

	}
    void TestBed::renderGUI(sf::RenderWindow& window, sf::Clock& clock)
    {
        const char* items[] = { "Bitmask" , "Bridge" , "Broadphase" , "Chain" , "Collision" , "Continuous", "Domino" , "Friction" ,
            "Geometry" , "Joints" , "Narrowphase" , "Newton's Cradle", "Position-Based Dynamics" , "Pendulum" , "AABB Raycast" , "Restitution" , "Sensor" , "Stacking" ,
            "Wrecking Ball", "Extended Position-Based Dynamics" };


        ImGui::SFML::Update(window, clock.restart());
        ImGui::Begin("Panel", nullptr, ImGuiWindowFlags_AlwaysAutoResize);

        ImGuiStyle& style = ImGui::GetStyle();
        style.WindowRounding = 5.0f;
        style.Colors[ImGuiCol_WindowBg] = ImVec4(0.1f, 0.1f, 0.1f, 0.55f);

        ImGui::PushItemWidth(150);
        ImGui::SetWindowPos("Panel", ImVec2(0, 0));
        ImGui::SetWindowSize("Panel", ImVec2(480, 800));
        ImGui::Text("Scenes");

        int oldItem = m_currentItem;
        ImGui::Combo("Current Scene", &m_currentItem, items, IM_ARRAYSIZE(items));
        if (oldItem != m_currentItem)
            change();
        

        ImGui::Separator();
        ImGui::Text("Sliders");
        ImGui::SliderInt("Position Iteration", &m_system.positionIteration(), 1, 20);
        ImGui::SliderInt("Velocity Iteration", &m_system.velocityIteration(), 1, 20);

        ImGui::SliderInt("Delta Time", &m_frequency, 30, 240);
        ImGui::SliderFloat("Contact Bias Factor", &m_system.maintainer().m_biasFactor, 0.01f, 0.1f);
        ImGui::SliderFloat("Contact Max Penetration", &m_system.maintainer().m_maxPenetration, 0.001f, 0.1f);

        ImGui::Separator();
        ImGui::Text("Switches");
        ImGui::Checkbox("Body Visible", &m_camera.bodyVisible());
        ImGui::Checkbox("AABB Visible", &m_camera.aabbVisible());
        ImGui::Checkbox("Joint Visible", &m_camera.jointVisible());
        ImGui::Checkbox("Grid Scale Line Visible", &m_camera.gridScaleLineVisible());
        ImGui::Checkbox("Tree Visible", &m_camera.treeVisible());
        ImGui::Checkbox("Contacts Visible", &m_camera.contactVisible());
        ImGui::Checkbox("User Draw Visible", &m_userDrawVisible);
        ImGui::Checkbox("Angle Visible", &m_camera.rotationLineVisible());
        ImGui::Checkbox("Center Visible", &m_camera.centerVisible());

        ImGui::Separator();
        ImGui::Text("Timestep");
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
        window.display();
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
        real dt = 1.0f / real(m_frequency);
    	m_system.step(dt);
	    if(m_currentFrame != nullptr)
		    m_currentFrame->update(dt);

    }
    void TestBed::simulate()
    {
        while (m_simulateWorkingState)
        {
            if (m_running)
                step();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    void TestBed::change()
    {
        m_simulateWorkingState = false;
        //stop thread
        m_physicsThread->wait();
        changeFrame();
        m_simulateWorkingState = true;
        m_physicsThread->launch();
    }
    void TestBed::changeFrame()
    {
        clearAll();
        switch (m_currentItem) {
            case 0:
                m_currentFrame = new BitmaskFrame(&m_system, &m_camera);
                break;
            case 1:
                m_currentFrame = new BridgeFrame(&m_system, &m_camera);
                break;
            case 2:
                m_currentFrame = new BroadPhaseFrame(&m_system, &m_camera);
                break;
            case 3:
                m_currentFrame = new ChainFrame(&m_system, &m_camera);
                break;
            case 4:
                m_currentFrame = new CollisionFrame(&m_system, &m_camera);
                break;
            case 5:
                m_currentFrame = new ContinuousFrame(&m_system, &m_camera);
                break;
            case 6:
                m_currentFrame = new DominoFrame(&m_system, &m_camera);
                break;
            case 7:
                m_currentFrame = new FrictionFrame(&m_system, &m_camera);
                break;
            case 8:
                m_currentFrame = new GeometryFrame(&m_system, &m_camera);
                break;
            case 9:
                m_currentFrame = new JointsFrame(&m_system, &m_camera);
                break;
            case 10:
                m_currentFrame = new NarrowphaseFrame(&m_system, &m_camera);
                break;
            case 11:
                m_currentFrame = new NewtonCradleFrame(&m_system, &m_camera);
                break;
            case 12:
                m_currentFrame = new PBDFrame(&m_system, &m_camera);
                break;
            case 13:
                m_currentFrame = new PendulumFrame(&m_system, &m_camera);
                break;
            case 14:
                m_currentFrame = new RaycastFrame(&m_system, &m_camera);
                break;
            case 15:
                m_currentFrame = new RestitutionFrame(&m_system, &m_camera);
                break;
            case 16:
                m_currentFrame = new SensorFrame(&m_system, &m_camera);
                break;
            case 17:
                m_currentFrame = new StackingFrame(&m_system, &m_camera);
                break;
            case 18:
                m_currentFrame = new WreckingBallFrame(&m_system, &m_camera);
                break;
            case 19:
                m_currentFrame = new XPBDFrame(&m_system, &m_camera);
                break;
            default:
                break;
        }
        if (m_currentFrame != nullptr)
            m_currentFrame->load();

    }
    void TestBed::clearAll()
    {
        m_system.world().clearAllBodies();
	    m_system.world().clearAllJoints();
        m_system.maintainer().clearAll();
        m_system.tree().clearAll();
        m_dbvh.cleanUp(m_dbvh.root());
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

