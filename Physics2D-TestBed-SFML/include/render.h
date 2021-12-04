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
    class RenderSFMLImpl
    {
    public:
        static void renderPoint(sf::RenderWindow& window, Camera& camera, const Vector2& point);
        static void renderLine(sf::RenderWindow& window, Camera& camera, const Vector2& p1, const Vector2& p2);
        static void renderPoints(sf::RenderWindow& window, Camera& camera, const std::vector<Vector2>& points);
        static void renderLines(sf::RenderWindow& window, Camera& camera, const std::vector<std::pair<Vector2, Vector2>>& lines);

        static void renderShape(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape);
        static void renderPolygon(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape);
        static void renderEdge(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape);
        static void renderRectangle(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape);
        static void renderCircle(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape);
        static void renderCapsule(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape);
        static void renderSector(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape);
        static void renderEllipse(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape);
        static void renderCurve(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape);
        static void renderAngleLine(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape);

        static void renderAABB(sf::RenderWindow& window, Camera& camera, const AABB& aabb);

        static void renderJoint(sf::RenderWindow& window, Camera& camera, Joint* joint);
        static void renderRotationJoint(sf::RenderWindow& window, Camera& camera, Joint* joint);
        static void renderDistanceJoint(sf::RenderWindow& window, Camera& camera, Joint* joint);
        static void renderPointJoint(sf::RenderWindow& window, Camera& camera, Joint* joint);


        static void renderOrientationJoint(sf::RenderWindow& window, Camera& camera, Joint* joint);
        static void renderPulleyJoint(sf::RenderWindow& window, Camera& camera, Joint* joint);
        static void renderPrismaticJoint(sf::RenderWindow& window, Camera& camera, Joint* joint);
        static void renderRevoluteJoint(sf::RenderWindow& window, Camera& camera, Joint* joint);
        static void renderWheelJoint(sf::RenderWindow& window, Camera& camera, Joint* joint);
    };
}
#endif