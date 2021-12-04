#include "include/render.h"
namespace Physics2D
{
	sf::Vector2f RenderSFMLImpl::toVector2f(const Vector2& vector)
	{
		return sf::Vector2f(vector.x, vector.y);
	}
	void RenderSFMLImpl::renderPoint(sf::RenderWindow& window, Camera& camera, const Vector2& point, const sf::Color& color, const int pointSize)
	{
		sf::CircleShape shape(pointSize);
		shape.setFillColor(color);
		shape.move(toVector2f(camera.worldToScreen(point)) - sf::Vector2f(RenderConstant::pointSize, RenderConstant::pointSize));
		window.draw(shape);
	}
	void RenderSFMLImpl::renderLine(sf::RenderWindow& window, Camera& camera, const Vector2& p1, const Vector2& p2, const sf::Color& color)
	{
		sf::Vertex line[] =
		{
			sf::Vertex(toVector2f(camera.worldToScreen(p1))),
			sf::Vertex(toVector2f(camera.worldToScreen(p2)))
		};
		line[0].color = color;
		line[1].color = color;
		window.draw(line, 2, sf::Lines);
	}
	void RenderSFMLImpl::renderPoints(sf::RenderWindow& window, Camera& camera, const std::vector<Vector2>& points, const sf::Color& color)
	{
		std::vector<sf::Vertex> vertices;
		vertices.reserve(points.size());
		for (auto& elem : points)
		{
			Vector2 screenPos = camera.worldToScreen(elem);
			sf::Vertex vertex;
			vertex.position = toVector2f(screenPos);
			vertex.color = color;
			vertices.emplace_back(vertex);
		}
		window.draw(&vertices[0], vertices.size(), sf::Points);
	}
	void RenderSFMLImpl::renderLines(sf::RenderWindow& window, Camera& camera, const std::vector<std::pair<Vector2, Vector2>>& lines, const sf::Color& color)
	{
		std::vector<sf::Vertex> vertices;
		vertices.reserve(lines.size() * 2);
		for (auto& elem : lines)
		{
			Vector2 screenPos = camera.worldToScreen(elem.first);
			sf::Vertex vertex;
			vertex.position = toVector2f(screenPos);
			vertex.color = color;
			vertices.emplace_back(vertex);

			screenPos = camera.worldToScreen(elem.second);
			vertex.position = toVector2f(screenPos);
			vertex.color = color;
			vertices.emplace_back(vertex);
		}
		window.draw(&vertices[0], vertices.size(), sf::Lines);
	}
	void RenderSFMLImpl::renderShape(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color)
	{
		switch (shape.shape->type())
		{
		case Shape::Type::Polygon:
		{
			renderPolygon(window, camera, shape, color);
			break;
		}
		case Shape::Type::Ellipse:
		{
			renderEllipse(window, camera, shape, color);
			break;
		}
		case Shape::Type::Circle:
		{
			renderCircle(window, camera, shape, color);
			break;
		}
		case Shape::Type::Curve:
		{
			renderCurve(window, camera, shape, color);
			break;
		}
		case Shape::Type::Edge:
		{
			renderEdge(window, camera, shape, color);
			break;
		}
		case Shape::Type::Capsule:
		{
			renderCapsule(window, camera, shape, color);
			break;
		}
		case Shape::Type::Sector:
		{
			renderSector(window, camera, shape, color);
			break;
		}
		default:
			break;
		}
	}
	void RenderSFMLImpl::renderPolygon(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color)
	{
		assert(shape.shape != nullptr);
		assert(shape.shape->type() == Shape::Type::Polygon);
		sf::ConvexShape convex;
		Polygon* polygon = dynamic_cast<Polygon*>(shape.shape);
		convex.setPointCount(polygon->vertices().size() - 1);
		for (size_t i = 0; i < polygon->vertices().size() - 1; ++i)
		{
			const Vector2 worldPos = Matrix2x2(shape.rotation).multiply(polygon->vertices()[i] * RenderConstant::scaleFactor) + shape.transform;
			const Vector2 screenPos = camera.worldToScreen(worldPos);
			convex.setPoint(i, toVector2f(screenPos));
		}
		sf::Color fillColor(color);
		fillColor.a = RenderConstant::fillAlpha;
		convex.setFillColor(fillColor);
		convex.setOutlineThickness(RenderConstant::borderSize);
		convex.setOutlineColor(color);
		window.draw(convex);
	}
	void RenderSFMLImpl::renderEdge(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color)
	{
		assert(shape.shape->type() == Shape::Type::Edge);
		Edge* edge = dynamic_cast<Edge*>(shape.shape);
		renderPoint(window, camera, edge->startPoint() + shape.transform, color);
		renderPoint(window, camera, edge->endPoint() + shape.transform, color);
		renderLine(window, camera, edge->startPoint() + shape.transform, edge->endPoint() + shape.transform, color);

		Vector2 center = (edge->startPoint() + edge->endPoint()) / 2.0f;
		center += shape.transform;
		renderLine(window, camera, center, center + 0.1f * edge->normal(), RenderConstant::materialYellow);
	}
	void RenderSFMLImpl::renderRectangle(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color)
	{
		assert(shape.shape != nullptr);
		assert(shape.shape->type() == Shape::Type::Polygon); 
		renderPolygon(window, camera, shape, color);
	}
	void RenderSFMLImpl::renderCircle(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color)
	{
		assert(shape.shape->type() == Shape::Type::Circle);
		const Circle* circle = dynamic_cast<Circle*>(shape.shape);
		const Vector2 screenPos = camera.worldToScreen(shape.transform);
		sf::CircleShape circleShape(circle->radius() * RenderConstant::scaleFactor * camera.meterToPixel());
		sf::Color fillColor(color);
		fillColor.a = RenderConstant::fillAlpha;
		circleShape.move(toVector2f(screenPos) - sf::Vector2f(circleShape.getRadius(), circleShape.getRadius()));
		circleShape.setFillColor(fillColor);
		circleShape.setOutlineThickness(RenderConstant::borderSize);
		circleShape.setOutlineColor(color);
		circleShape.setPointCount(RenderConstant::basicCirclePointCount + camera.meterToPixel());
		window.draw(circleShape);
	}
	void RenderSFMLImpl::renderCapsule(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color)
	{
		assert(shape.shape->type() == Shape::Type::Capsule);
		std::vector<sf::Vertex> vertices;

		const Capsule* capsule = dynamic_cast<Capsule*>(shape.shape);
		const Vector2 screenPos = camera.worldToScreen(shape.transform);
		int pointCounts = (RenderConstant::basicCirclePointCount + camera.meterToPixel()) / 4;
		sf::Vertex centerVertex = toVector2f(screenPos);
		sf::Color fillColor(color);
		fillColor.a = RenderConstant::fillAlpha;
		centerVertex.color = fillColor;
		vertices.emplace_back(centerVertex);
		auto sampling = [&](const Vector2& center, const real& radius, const real& startRadians, const real& endRadians)
		{
			real step = (endRadians - startRadians) / float(pointCounts);
			for (real radian = startRadians; radian <= endRadians; radian += step)
			{
				Vector2 point(radius * Math::cosx(radian), radius * Math::sinx(radian));
				point += center;
				const Vector2 worldPos = Matrix2x2(shape.rotation).multiply(point * RenderConstant::scaleFactor) + shape.transform;
				const Vector2 screenPos = camera.worldToScreen(worldPos);
				sf::Vertex vertex;
				vertex.position = toVector2f(screenPos);
				vertex.color = fillColor;
				vertices.emplace_back(vertex);
			}
		};
		if (capsule->width() > capsule->height())
		{
			real radius = capsule->height() / 2.0f;
			sampling((capsule->bottomLeft() + capsule->topLeft()) / 2.0f, radius, Math::degreeToRadian(90), Math::degreeToRadian(270));
			sampling((capsule->topRight() + capsule->bottomRight()) / 2.0f, radius, Math::degreeToRadian(270), Math::degreeToRadian(450));
		}
		else
		{
			real radius = capsule->width() / 2.0f;
			sampling((capsule->topLeft() + capsule->topRight()) / 2.0f, radius, Math::degreeToRadian(0), Math::degreeToRadian(180));
			sampling((capsule->bottomLeft() + capsule->bottomRight()) / 2.0f, radius, Math::degreeToRadian(180), Math::degreeToRadian(360));
		}
		vertices.emplace_back(vertices[1]);
		window.draw(&vertices[0], vertices.size(), sf::TriangleFan);
		for (auto& elem : vertices)
			elem.color = color;
		window.draw(&vertices[1], vertices.size() - 1, sf::LinesStrip);
	}
	void RenderSFMLImpl::renderSector(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color)
	{
		assert(shape.shape->type() == Shape::Type::Sector);
		std::vector<sf::Vertex> vertices;

	}
	void RenderSFMLImpl::renderEllipse(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color)
	{
		assert(shape.shape->type() == Shape::Type::Ellipse);
		std::vector<sf::Vertex> vertices;

		const Ellipse* ellipse = dynamic_cast<Ellipse*>(shape.shape);
		const Vector2 screenPos = camera.worldToScreen(shape.transform);
		int pointCounts = (RenderConstant::basicCirclePointCount + camera.meterToPixel()) / 2;

		sf::Vertex centerVertex = toVector2f(screenPos);
		sf::Color fillColor(color);
		fillColor.a = RenderConstant::fillAlpha;
		centerVertex.color = fillColor;
		vertices.emplace_back(centerVertex);

		real step = Constant::DoublePi / float(pointCounts);
		real innerRadius, outerRadius;
		innerRadius = ellipse->A();
		outerRadius = ellipse->B();
		if (ellipse->A() > ellipse->B())
		{
			innerRadius = ellipse->B();
			outerRadius = ellipse->A();
		}
		for (real radian = 0; radian <= Constant::DoublePi; radian += step)
		{
			Vector2 point(outerRadius * Math::cosx(radian), innerRadius * Math::sinx(radian));

			const Vector2 worldPos = Matrix2x2(shape.rotation).multiply(point * RenderConstant::scaleFactor) + shape.transform;
			const Vector2 screenPos = camera.worldToScreen(worldPos);
			sf::Vertex vertex;
			vertex.position = toVector2f(screenPos);
			vertex.color = fillColor;
			vertices.emplace_back(vertex);
		}
		vertices.emplace_back(vertices[1]);
		window.draw(&vertices[0], vertices.size(), sf::TriangleFan);
		for (auto& elem : vertices)
			elem.color = color;
		window.draw(&vertices[1], vertices.size() - 1, sf::LinesStrip);
	}
	void RenderSFMLImpl::renderCurve(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color)
	{
		assert(shape.shape->type() == Shape::Type::Curve);
		std::vector<sf::Vertex> vertices;

	}
	void RenderSFMLImpl::renderAngleLine(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color)
	{
		sf::Color colorX(139, 195, 74);
		sf::Color colorY(255, 235, 59);
		colorX.a = 204;
		colorY.a = 204;
		Vector2 xP(0.1f, 0);
		Vector2 yP(0, 0.1f);
		Vector2 mc = Matrix2x2(shape.rotation).multiply(shape.shape->center());
		xP = Matrix2x2(shape.rotation).multiply(xP) + shape.transform + mc;
		yP = Matrix2x2(shape.rotation).multiply(yP) + shape.transform + mc;
		renderLine(window, camera, shape.transform + mc, xP, colorX);
		renderLine(window, camera, shape.transform + mc, yP, colorY);
	}
	void RenderSFMLImpl::renderAABB(sf::RenderWindow& window, Camera& camera, const AABB& aabb, const sf::Color& color)
	{
		Vector2 aabbSize(aabb.width, aabb.height);
		aabbSize *= camera.meterToPixel();
		sf::RectangleShape shape(toVector2f(aabbSize));
		shape.move(toVector2f(camera.worldToScreen(aabb.topLeft())));
		shape.setFillColor(sf::Color::Transparent);
		shape.setOutlineThickness(RenderConstant::borderSize);
		shape.setOutlineColor(color);
		window.draw(shape);
	}
	void RenderSFMLImpl::renderJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color)
	{
		switch (joint->type())
		{
		case JointType::Rotation:
		{
			renderRotationJoint(window, camera, joint, color);
			break;
		}
		case JointType::Distance:
		{
			renderDistanceJoint(window, camera, joint, color);
			break;
		}
		case JointType::Point:
		{
			renderPointJoint(window, camera, joint, color);
			break;
		}
		case JointType::Orientation:
		{
			renderOrientationJoint(window, camera, joint, color);
			break;
		}
		case JointType::Pulley:
		{
			renderPulleyJoint(window, camera, joint, color);
			break;
		}
		case JointType::Prismatic:
		{
			renderPrismaticJoint(window, camera, joint, color);
			break;
		}
		case JointType::Revolute:
		{
			renderRevoluteJoint(window, camera, joint, color);
			break;
		}
		case JointType::Wheel:
		{
			renderWheelJoint(window, camera, joint, color);
			break;
		}
		default:
			break;
		}
	}
	void RenderSFMLImpl::renderRotationJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color)
	{
	}
	void RenderSFMLImpl::renderDistanceJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color)
	{
		assert(joint != nullptr);
		DistanceJoint* distanceJoint = dynamic_cast<DistanceJoint*>(joint);
		Vector2 pa = distanceJoint->primitive().bodyA->toWorldPoint(distanceJoint->primitive().localPointA);
		Vector2 pb = distanceJoint->primitive().targetPoint;
		Vector2 n = (pa - pb).normal();
		Vector2 minPoint = n * distanceJoint->primitive().minDistance + pb;
		Vector2 maxPoint = n * distanceJoint->primitive().maxDistance + pb;
		sf::Color minColor = RenderConstant::materialBlue;
		sf::Color maxColor = RenderConstant::materialRed;
		minColor.a = 204;
		maxColor.a = 204;
		renderPoint(window, camera, pa, RenderConstant::materialGray);
		renderPoint(window, camera, pb, RenderConstant::materialGray);
		renderPoint(window, camera, minPoint, minColor);
		renderPoint(window, camera, maxPoint, maxColor);
		sf::Color lineColor = RenderConstant::materialGray;
		lineColor.a = 150;
		renderLine(window, camera, pa, pb, lineColor);

	}
	void RenderSFMLImpl::renderPointJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color)
	{
		assert(joint != nullptr);
		PointJoint* pointJoint = dynamic_cast<PointJoint*>(joint);
		Vector2 pa = pointJoint->primitive().bodyA->toWorldPoint(pointJoint->primitive().localPointA);
		Vector2 pb = pointJoint->primitive().targetPoint;

		sf::Color gray = RenderConstant::materialGray;
		sf::Color green = sf::Color::Green;
		gray.a = 204;
		green.a = 78;

		renderPoint(window, camera, pa, gray, 2);
		renderPoint(window, camera, pb, gray, 2);
		renderLine(window, camera, pa, pb, green);
	}
	void RenderSFMLImpl::renderOrientationJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color)
	{
		assert(joint != nullptr);
		OrientationJoint* pointJoint = dynamic_cast<OrientationJoint*>(joint);
		Vector2 pa = pointJoint->primitive().bodyA->position();
		Vector2 pb = pointJoint->primitive().targetPoint;

		sf::Color gray = RenderConstant::materialGray;
		sf::Color green = sf::Color::Green;
		gray.a = 204;
		green.a = 78;

		renderPoint(window, camera, pa, gray, 2);
		renderPoint(window, camera, pb, gray, 2);
		renderLine(window, camera, pa, pb, green);
	}
	void RenderSFMLImpl::renderPulleyJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color)
	{
	}
	void RenderSFMLImpl::renderPrismaticJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color)
	{
	}
	void RenderSFMLImpl::renderRevoluteJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color)
	{
		assert(joint != nullptr);
		RevoluteJoint* revoluteJoint = dynamic_cast<RevoluteJoint*>(joint);
		Vector2 pa = revoluteJoint->primitive().bodyA->toWorldPoint(revoluteJoint->primitive().localPointA);
		Vector2 pb = revoluteJoint->primitive().bodyB->toWorldPoint(revoluteJoint->primitive().localPointB);

		sf::Color gray = RenderConstant::materialGray;
		sf::Color green = sf::Color::Green;
		gray.a = 204;
		green.a = 78;

		renderPoint(window, camera, pa, gray, 2);
		renderPoint(window, camera, pb, gray, 2);
		renderLine(window, camera, pa, pb, green);
	}
	void RenderSFMLImpl::renderWheelJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color)
	{
	}
}