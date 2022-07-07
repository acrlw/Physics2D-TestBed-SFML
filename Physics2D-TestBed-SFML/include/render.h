#ifndef RENDER_H
#define RENDER_H

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Graphics.hpp>

#include <dependencies/Physics2D/include/physics2d.h>

#include "camera.h"

namespace Physics2D
{
    namespace RenderConstant
    {
        const int PointSize = 2;
        const int BorderSize = 1;
        const int FillAlpha = 38;
        const int BasicCirclePointCount = 60;
        const real BasicDashLength = 2;
        const sf::Color MaterialYellow = sf::Color(255, 235, 59);
        const sf::Color MaterialRed = sf::Color(244, 67, 54);
        const sf::Color MaterialBlue = sf::Color(68, 138, 255);
        const sf::Color MaterialPink = sf::Color(233, 30, 99);
        const sf::Color MaterialDarkGreen = sf::Color(44, 113, 48);
        const sf::Color MaterialGray = sf::Color(158, 158, 158);
        const sf::Color MaterialOrange = sf::Color(255, 138, 101);
        const sf::Color MaterialTeal = sf::Color(29, 233, 182);
        static real ScaleFactor = 0.97f;
    }
    class RenderSFMLImpl
    {
    public:
        static sf::Vector2f toVector2f(const Vector2& vector);
        static void renderPoint(sf::RenderWindow& window, Camera& camera, const Vector2& point, const sf::Color& color, const int pointSize = RenderConstant::PointSize);
        static void renderLine(sf::RenderWindow& window, Camera& camera, const Vector2& p1, const Vector2& p2, const sf::Color& color);

    	static void renderPoints(sf::RenderWindow& window, Camera& camera, const std::vector<Vector2>& points, const sf::Color& color);
        
    	static void renderLines(sf::RenderWindow& window, Camera& camera, const std::vector<std::pair<Vector2, Vector2>>& lines, const sf::Color& color);
        
        static void renderShape(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color);
        static void renderPolygon(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color);
        static void renderEdge(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color);
        static void renderRectangle(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color);
        static void renderCircle(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color);
        static void renderCapsule(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color);
        static void renderSector(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color);
        static void renderEllipse(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color);
        static void renderCurve(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color);
        static void renderAngleLine(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color);

        static void renderBody(sf::RenderWindow& window, Camera& camera, Body* body, const sf::Color& color);
        static void renderAABB(sf::RenderWindow& window, Camera& camera, const AABB& aabb, const sf::Color& color);

        static void renderJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color);
        static void renderRotationJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color);
        static void renderDistanceJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color);
        static void renderPointJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color);


        static void renderOrientationJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color);
        static void renderPulleyJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color);
        static void renderPrismaticJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color);
        static void renderRevoluteJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color);
        static void renderWheelJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color);
    };
}
#endif