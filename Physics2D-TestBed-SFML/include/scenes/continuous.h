#ifndef PHYSICS2D_SCENES_CONTINUOUS_H
#define PHYSICS2D_SCENES_CONTINUOUS_H
#include "frame.h"

namespace Physics2D
{
    class ContinuousFrame : public Frame
    {
    public:
        ContinuousFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
            Tree* tree, UniformGrid* grid, Camera* camera) : Frame("Continuous", world, maintainer, tree, grid, camera)
        {

        }
        void load() override
        {

            block.set(300, 3.0f);
            edge.set({ -200, 0 }, { 149.9f, 0 });
            rect.set(1.0f, 1.0f);
            circle.setRadius(0.1f);
            stick.set(2.0f, 0.5f);
            wall.set(60.0, 400.0f);

            Body* ground = m_world->createBody();
            ground->setShape(&block);
            ground->position().set({ 0, -1.5});
            ground->setMass(100000);
            ground->setType(Body::BodyType::Static);
            ground->setFriction(0.1f);
            m_tree->insert(ground);

            for (real j = 0; j < 20.0f; j += 1.0f)
            {
                for (real i = 0; i < 3.0; i += 1.0f)
                {
                    Body* body = m_world->createBody();
                    body->position().set({ i * 1.05f - 2.0f, j * 1.05f - ground->position().y + 0.55f });
                    body->setShape(&rect);
                    body->rotation() = 0.0f;
                    body->setMass(1.0f);
                    body->setType(Body::BodyType::Dynamic);
                    body->setFriction(0.1f);
                    body->setRestitution(0.0f);
                    m_tree->insert(body);
                }
            }

            Body* bullet = m_world->createBody();
            bullet->setShape(&stick);
            bullet->position().set({ -100.0f, 6.5f });
            bullet->setType(Body::BodyType::Bullet);
            bullet->setMass(5.0f);
            bullet->velocity().set({ 1000.0f, 0.0f });
            bullet->angularVelocity() = -200.0f;
            m_tree->insert(bullet);

            Body* wallBody = m_world->createBody();
            wallBody->setShape(&wall);
            wallBody->position().set(180.0f, 0.0f);
            wallBody->setMass(100000);
            wallBody->setFriction(0.1f);
            wallBody->setType(Body::BodyType::Static);

            m_tree->insert(wallBody);

            //m_camera->setTargetBody(bullet);
            //m_camera->setMeterToPixel(5);

        }
        void render(sf::RenderWindow& window) override
        {

        }
        void release()override
        {
            m_camera->setTargetBody(nullptr);
        }
    private:
        Edge edge;
        Rectangle block;
        Rectangle rect;
        Rectangle stick;
        Rectangle wall;
        Circle circle;
    };
}
#endif
