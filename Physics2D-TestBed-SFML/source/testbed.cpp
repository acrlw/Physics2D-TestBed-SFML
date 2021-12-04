#include "..\include\testbed.h"
namespace Physics2D
{
	void TestBed::exec()
	{
        // create the window
        sf::ContextSettings settings;
        settings.antialiasingLevel = 8;
        sf::RenderWindow window(sf::VideoMode(1920, 1080), "Testbed", sf::Style::Default, settings);
        window.setFramerateLimit(60);
        ImGui::SFML::Init(window);

        window.setActive(false);
        //sf::Thread thread(&TestBed::render, &window);
        //thread.launch();
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
                    window.close();
                    break;
                }
                default:
                    break;
                }

                if (event.type == sf::Event::Closed) {
                    window.close();
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
        ImGui::Combo("Current Scene", &m_currentItem, items, IM_ARRAYSIZE(items));

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

    }
    void TestBed::restart()
    {
    }
    void TestBed::step()
    {
    }
}

