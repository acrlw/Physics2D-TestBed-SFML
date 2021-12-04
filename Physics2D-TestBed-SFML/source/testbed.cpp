#include "..\include\testbed.h"
namespace Physics2D
{
    TestBed::TestBed()
    {
        m_world.setEnableGravity(true);
        m_world.setGravity({ 0, -9.8f });
        m_world.setLinearVelocityDamping(0.1f);
        m_world.setAirFrictionCoefficient(0.8f);
        m_world.setAngularVelocityDamping(0.1f);
        m_world.setEnableDamping(true);
        m_world.setPositionIteration(8);
        m_world.setVelocityIteration(6);

        m_pointJointPrimitive.bodyA = nullptr;
        m_mouseJoint = m_world.createJoint(m_pointJointPrimitive);
        m_mouseJoint->setActive(false);
    }
    TestBed::~TestBed()
    {

    }
    void TestBed::exec()
	{
        // create the window
        sf::ContextSettings settings;
        settings.antialiasingLevel = 8;
        sf::RenderWindow window(sf::VideoMode(1920, 1080), "Testbed", sf::Style::Default, settings);
        window.setFramerateLimit(60);
        ImGui::SFML::Init(window);

        window.setActive(false);

        sf::Thread physicsThread(&TestBed::simulate, this);
        physicsThread.launch();

        sf::Clock deltaClock;
        while (window.isOpen())
        {
            sf::Event event;
            while (window.pollEvent(event)) {
                ImGui::SFML::ProcessEvent(event);

                switch (event.type)
                {
                case sf::Event::Closed:
                {
                    physicsThread.terminate();
                    window.close();
                    break;
                }
                case sf::Event::KeyReleased:
                {
                    switch (event.key.code)
                    {
                    case sf::Keyboard::Space:
                    {
                        std::cout << "pause" << std::endl;
                        break;
                    }
                    default:
                        break;
                    }
                    if (m_currentFrame != nullptr)
                        m_currentFrame->onKeyRelease(event);
                }
                case sf::Event::MouseButtonPressed:
                {
                    if (m_currentFrame != nullptr)
                        m_currentFrame->onMousePress(event);
                    break;
                }
                case sf::Event::MouseButtonReleased:
                {
                    if (m_currentFrame != nullptr)
                        m_currentFrame->onMouseRelease(event);
                    break;
                }
                case sf::Event::MouseMoved:
                {
                    if (m_currentFrame != nullptr)
                        m_currentFrame->onMouseMove(event);
                    break;
                }
                case sf::Event::MouseWheelScrolled:
                {
                    std::cout << "wheel movement: " << event.mouseWheelScroll.wheel << std::endl;
                    break;
                }
                default:
                    break;
                }
            }



            render(window);
            renderGUI(window, deltaClock);

        }
        ImGui::SFML::Shutdown();

	}
    void TestBed::render(sf::RenderWindow& window)
    {
        window.clear(sf::Color(50, 50, 50));

        sf::CircleShape shape(50.f);
        sf::Color color = sf::Color::Green;
        color.a = 38;
        shape.setFillColor(color);

        // set a 10-pixel wide orange outline
        shape.setOutlineThickness(1.f);
        color.a = 255;
        shape.setOutlineColor(color);

        window.draw(shape);
    }
    void TestBed::renderGUI(sf::RenderWindow& window, sf::Clock& clock)
    {
        const char* items[] = { "Bitmask" , "Bridge" , "Broadphase" , "Chain" , "Collision" , "Domino" , "Friction" ,
            "Geometry" , "Joints" , "Narrowphase" , "Newton's Cradle" , "Pendulum" , "AABB Raycast" , "Restitution" , "Sensor" , "Stacking" ,
            "Wrecking Ball" };


        ImGui::SFML::Update(window, clock.restart());
        ImGui::Begin("Panel", NULL, ImGuiWindowFlags_AlwaysAutoResize);

        ImGuiStyle& style = ImGui::GetStyle();
        style.WindowRounding = 5.0f;
        style.Colors[ImGuiCol_WindowBg] = ImVec4(0.1f, 0.1f, 0.1f, 0.55f);

        ImGui::PushItemWidth(300);
        ImGui::SetWindowPos("Panel", ImVec2(0, 0));
        ImGui::SetWindowSize("Panel", ImVec2(480, 800));
        ImGui::Text("Scenes");

        int oldItem = m_currentItem;
        ImGui::Combo("Current Scene", &m_currentItem, items, IM_ARRAYSIZE(items));
        if (oldItem != m_currentItem)
        {
            std::cout << "change scene" << std::endl;
        }

        ImGui::Separator();
        ImGui::Text("Sliders");
        ImGui::SliderInt("Position Iteration", &m_positionIteration, 1, 20);
        ImGui::SliderInt("Velocity Iteration", &m_velocityIteration, 1, 20);
        ImGui::SliderInt("Delta Time", &m_frequency, 30, 240);
        ImGui::SliderFloat("Contact Bias Factor", &m_contactBiasFactor, 0.01f, 0.1f);

        ImGui::Separator();
        ImGui::Text("Switches");
        ImGui::Checkbox("Body Visible", &m_bodyVisible);
        ImGui::Checkbox("AABB Visible", &m_aabbVisible);
        ImGui::Checkbox("Joint Visible", &m_jointVisible);
        ImGui::Checkbox("Grid Scale Line Visible", &m_gridVisible);
        ImGui::Checkbox("Tree Visible", &m_treeVisible);
        ImGui::Checkbox("Contacts Visible", &m_contactsVisible);
        ImGui::Checkbox("Axis Visible", &m_axisVisible);
        ImGui::Checkbox("User Draw Visible", &m_userDrawVisible);
        ImGui::Checkbox("Angle Visible", &m_angleVisible);
        ImGui::Checkbox("Center Visible", &m_centerVisible);

        ImGui::Separator();
        ImGui::Text("Timestep");
        if (ImGui::Button("Pause", ImVec2(-FLT_MIN, 0.0f)))
            pause();
        if (ImGui::Button("Step", ImVec2(-FLT_MIN, 0.0f)))
            step();
        if (ImGui::Button("Restart", ImVec2(-FLT_MIN, 0.0f)))
            restart();


        ImGui::End();
        ImGui::SFML::Render(window);
        window.display();
    }
    void TestBed::pause()
    {
        m_running = !m_running;
    }
    void TestBed::restart()
    {
        changeFrame();
    }
    void TestBed::step()
    {
        real dt = 1.0f / m_frequency;

        for (auto& elem : m_world.bodyList())
            m_tree.update(elem.get());

        if (m_currentFrame != nullptr)
            m_currentFrame->update(dt);

        m_world.stepVelocity(dt);

        auto potentialList = m_tree.generate();
        for (auto pair : potentialList)
        {
            auto result = Detector::detect(pair.first, pair.second);
            if (result.isColliding)
                m_maintainer.add(result);
        }
        m_maintainer.clearInactivePoints();
        m_world.prepareVelocityConstraint(dt);

        for (int i = 0; i < m_world.velocityIteration(); ++i)
        {
            m_world.solveVelocityConstraint(dt);
            m_maintainer.solveVelocity(dt);
        }

        m_world.stepPosition(dt);

        for (int i = 0; i < m_world.positionIteration(); ++i)
        {
            m_maintainer.solvePosition(dt);
            m_world.solvePositionConstraint(dt);
        }
        m_maintainer.deactivateAllPoints();

    }
    void TestBed::simulate()
    {
        while (true)
        {
            if (m_running)
                step();
            std::cout << "tick, running:" << m_running << std::endl;
            sf::sleep(sf::milliseconds(1000));
        }
    }
    void TestBed::changeFrame()
    {
        clearAll();
        switch (m_currentItem)
        {
        case 0:
            m_currentFrame = new BitmaskFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh, &m_camera);
            break;
        case 1:
            m_currentFrame = new BridgeFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh, &m_camera);
            break;
        case 2:
            m_currentFrame = new BroadPhaseFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh, &m_camera);
            break;
        case 3:
            m_currentFrame = new ChainFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh, &m_camera);
            break;
        case 4:
            m_currentFrame = new CollisionFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh, &m_camera);
            break;
        case 5:
            m_currentFrame = new DominoFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh, &m_camera);
            break;
        case 6:
            m_currentFrame = new FrictionFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh, &m_camera);
            break;
        case 7:
            m_currentFrame = new GeometryFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh, &m_camera);
            break;
        case 8:
            m_currentFrame = new JointsFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh, &m_camera);
            break;
        case 9:
            m_currentFrame = new NarrowphaseFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh, &m_camera);
            break;
        case 10:
            m_currentFrame = new NewtonCradleFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh, &m_camera);
            break;
        case 11:
            m_currentFrame = new PendulumFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh, &m_camera);
            break;
        case 12:
            m_currentFrame = new RaycastFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh, &m_camera);
            break;
        case 13:
            m_currentFrame = new RestitutionFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh, &m_camera);
            break;
        case 14:
            m_currentFrame = new SensorFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh, &m_camera);
            break;
        case 15:
            m_currentFrame = new StackingFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh, &m_camera);
            break;
        case 16:
            m_currentFrame = new WreckingBallFrame(&m_world, &m_maintainer, &m_tree, &m_dbvh, &m_camera);
            break;
        default:
            break;
        }
        if (m_currentFrame != nullptr)
            m_currentFrame->load();
    }
    void TestBed::clearAll()
    {
        m_world.clearAllBodies();
        m_world.clearAllJoints();
        m_maintainer.clearAll();
        m_tree.clearAll();
        m_dbvh.cleanUp(m_dbvh.root());
        m_pointJointPrimitive.bodyA = nullptr;
        m_mouseJoint = m_world.createJoint(m_pointJointPrimitive);
        m_mouseJoint->setActive(false);
        if (m_currentFrame != nullptr)
        {
            m_currentFrame->release();
            delete m_currentFrame;
            m_currentFrame = nullptr;
        }
    }
}

