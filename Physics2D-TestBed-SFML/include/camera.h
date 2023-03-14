#ifndef PHYSICS2D_UTILS_CAMERA_H
#define PHYSICS2D_UTILS_CAMERA_H

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Graphics.hpp>

#include "../dependencies/Physics2D/physics2d.h"
namespace Physics2D
{
    class Camera
    {
    public:
        enum class EasingType
        {
            Lerp,
            Exponential,
            Uniform
        };
        struct Viewport
        {
            Viewport() = default;
            Viewport(const Vector2& top_left, const Vector2& bottom_right) : topLeft(top_left), bottomRight(bottom_right) {}
            Vector2 topLeft = { 0, 0 };
            Vector2 bottomRight = { 800, 600 };
            real width();
            real height();
            void setWidth(const real& width);
            void setHeight(const real& height);
            void set(const real& width, const real& height);
        };
        Camera() = default;
        void render(sf::RenderWindow& window);

        bool& aabbVisible();
        bool& jointVisible();
        bool& bodyVisible();
        bool& gridScaleLineVisible();
        bool& visible();
        bool& treeVisible();
        bool& rotationLineVisible();
        bool& centerVisible();
        bool& contactVisible();
        bool& dbvhVisible();

        real axisPointCount()const;
        void setAxisPointCount(real count);

        real meterToPixel() const;
        void setMeterToPixel(const real& meterToPixel);

        Vector2 transform() const;
        void setTransform(const Vector2& transform);

        void setWorld(PhysicsWorld* world);
        PhysicsWorld* world()const;

        Body* targetBody() const;
        void setTargetBody(Body* targetBody);

        real zoomFactor() const;
        void setZoomFactor(const real& zoomFactor);


        Viewport viewport() const;
        void setViewport(const Viewport& viewport);

        Vector2 worldToScreen(const Vector2& pos)const;
        Vector2 screenToWorld(const Vector2& pos)const;

        DBVH* dbvh()const;
        void setDbvh(DBVH* dbvh);

        Tree* tree()const;
        void setTree(Tree* tree);


        real deltaTime()const;
        void setDeltaTime(const real& deltaTime);


        ContactMaintainer* maintainer()const;
        void setContactMaintainer(ContactMaintainer* maintainer);

        EasingType easingType()const;
        void setEasingType(EasingType type);

    private:
        void drawGridScaleLine(sf::RenderWindow& window);
        void drawDbvh(DBVH::Node* node, sf::RenderWindow& window);
        void drawTree(int nodeIndex, sf::RenderWindow& window);
        void drawContacts(sf::RenderWindow& window);

        bool m_visible = true;
        bool m_aabbVisible = false;
        bool m_jointVisible = true;
        bool m_bodyVisible = true;
        bool m_dbvhVisible = false;
        bool m_treeVisible = false;
        bool m_gridScaleLineVisible = false;
        bool m_rotationLineVisible = false;
        bool m_centerVisible = false;
        bool m_contactVisible = false;


        real m_meterToPixel = 50.0f;
        real m_pixelToMeter = 0.02f;

        real m_targetMeterToPixel = 80.0f;
        real m_targetPixelToMeter = 0.02f;

        Vector2 m_transform;
        Vector2 m_origin;
        Viewport m_viewport;
        PhysicsWorld* m_world = nullptr;
        Body* m_targetBody = nullptr;
        DBVH* m_dbvh = nullptr;
        Tree* m_tree = nullptr;
        ContactMaintainer* m_maintainer = nullptr;

        real m_zoomFactor = 1.0f;
        real m_restitution = 2.0f;
        real m_deltaTime = 15.0f;
        real m_axisPointCount = 50.0f;

        EasingType m_easingType = EasingType::Exponential;

    };


}
#endif
