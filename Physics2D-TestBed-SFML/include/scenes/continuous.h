#ifndef PHYSICS2D_SCENES_CONTINUOUS_H
#define PHYSICS2D_SCENES_CONTINUOUS_H
#include "./include/frame.h"

namespace Physics2D
{
    class ContinuousFrame : public Frame
    {
    public:
        ContinuousFrame(PhysicsWorld* world, ContactMaintainer* maintainer,
                      Tree* tree, DBVH* dbvh, Camera* camera) : Frame("Geometry", world, maintainer, tree, dbvh, camera)
        {

        }
        void load() override
        {
            edge.set({ -100, 0 }, { 100, 0 });
            rect.set(1.0f, 1.0f);
            circle.setRadius(0.1f);
            stick.set(8.0f, 0.5f);

            Body* ground = m_world->createBody();
            ground->setShape(&edge);
            ground->position().set({ 0,  });
            ground->setMass(Constant::Max);
            ground->setType(Body::BodyType::Static);
            m_tree->insert(ground);

            for(real i = 1.0f; i <= 10.0f; i += 1.1f)
            {
                Body* body = m_world->createBody();
                body->setShape(&rect);
                body->position().set({0, i});
                body->setType(Body::BodyType::Dynamic);
                body->setMass(5.0f);
                m_tree->insert(body);
            }

            Body* bullet = m_world->createBody();
            bullet->setShape(&circle);
            bullet->position().set({-20.0f, 5.3f});
            bullet->setType(Body::BodyType::Bullet);
            bullet->setMass(0.8f);
            bullet->velocity().set({1000.0f, 0.0f});
            //bullet->angularVelocity() = 1000.0f;
            m_tree->insert(bullet);

        }
        void render(sf::RenderWindow& window) override
        {

        }
    private:
        Edge edge;
        Rectangle rect;
        Rectangle stick;
        Circle circle;
    };
}
#endif
