#include "render.h"
namespace Physics2D
{
	sf::Vector2f RenderSFMLImpl::toVec2f(const Vec2& vector)
	{
		return sf::Vector2f(vector.x, vector.y);
	}
	void RenderSFMLImpl::renderPoint(sf::RenderWindow& window, Camera& camera, const Vec2& point, const sf::Color& color, const int pointSize)
	{
		sf::CircleShape shape(pointSize);
		shape.setFillColor(color);
		shape.move(toVec2f(camera.worldToScreen(point)) - sf::Vector2f(RenderConstant::PointSize, RenderConstant::PointSize));
		window.draw(shape);
	}
	void RenderSFMLImpl::renderLine(sf::RenderWindow& window, Camera& camera, const Vec2& p1, const Vec2& p2, const sf::Color& color)
	{
		sf::Vertex line[] =
		{
			sf::Vertex(toVec2f(camera.worldToScreen(p1))),
			sf::Vertex(toVec2f(camera.worldToScreen(p2)))
		};
		line[0].color = color;
		line[1].color = color;
		window.draw(line, 2, sf::Lines);
	}
	void RenderSFMLImpl::renderPoints(sf::RenderWindow& window, Camera& camera, const Container::Vector<Vec2>& points, const sf::Color& color)
	{
		Container::Vector<sf::Vertex> vertices;
		vertices.reserve(points.size());
		for (auto& elem : points)
		{
			Vec2 screenPos = camera.worldToScreen(elem);
			sf::Vertex vertex;
			vertex.position = toVec2f(screenPos);
			vertex.color = color;
			vertices.emplace_back(vertex);
		}
		window.draw(&vertices[0], vertices.size(), sf::Points);
	}



	void RenderSFMLImpl::renderLines(sf::RenderWindow& window, Camera& camera, const Container::Vector<std::pair<Vec2, Vec2>>& lines, const sf::Color& color)
	{
		Container::Vector<sf::Vertex> vertices;
		vertices.reserve(lines.size() * 2);
		for (auto& elem : lines)
		{
			Vec2 screenPos = camera.worldToScreen(elem.first);
			sf::Vertex vertex;
			vertex.position = toVec2f(screenPos);
			vertex.color = color;
			vertices.emplace_back(vertex);

			screenPos = camera.worldToScreen(elem.second);
			vertex.position = toVec2f(screenPos);
			vertex.color = color;
			vertices.emplace_back(vertex);
		}
		window.draw(&vertices[0], vertices.size(), sf::Lines);
	}


	void RenderSFMLImpl::renderShape(sf::RenderWindow& window, Camera& camera, const Transform& transform, Shape* shape, const sf::Color& color)
	{
		switch (shape->type())
		{
		case Shape::Type::Polygon:
		{
			renderPolygon(window, camera, transform, shape, color);
			break;
		}
		case Shape::Type::Ellipse:
		{
			renderEllipse(window, camera, transform, shape, color);
			break;
		}
		case Shape::Type::Circle:
		{
			renderCircle(window, camera, transform, shape, color);
			break;
		}
		case Shape::Type::Curve:
		{
			renderCurve(window, camera, transform, shape, color);
			break;
		}
		case Shape::Type::Edge:
		{
			renderEdge(window, camera, transform, shape, color);
			break;
		}
		case Shape::Type::Capsule:
		{
			renderCapsule(window, camera, transform, shape, color);
			break;
		}
		case Shape::Type::Sector:
		{
			renderSector(window, camera, transform, shape, color);
			break;
		}
		default:
			break;
		}
	}
	void RenderSFMLImpl::renderPolygon(sf::RenderWindow& window, Camera& camera, const Transform& transform, Shape* shape, const sf::Color& color)
	{
		assert(shape != nullptr);
		assert(shape->type() == Shape::Type::Polygon);
		sf::ConvexShape convex;
		Polygon* polygon = static_cast<Polygon*>(shape);
		convex.setPointCount(polygon->vertices().size() - 1);
		for (size_t i = 0; i < polygon->vertices().size() - 1; ++i)
		{
			const Vec2 worldPos = transform.transform(polygon->vertices()[i] * RenderConstant::ScaleFactor);
			const Vec2 screenPos = camera.worldToScreen(worldPos);
			convex.setPoint(i, toVec2f(screenPos));
		}
		sf::Color fillColor(color);
		fillColor.a = RenderConstant::FillAlpha;
		convex.setFillColor(fillColor);
		convex.setOutlineThickness(RenderConstant::BorderSize);
		convex.setOutlineColor(color);
		window.draw(convex);
	}
	void RenderSFMLImpl::renderEdge(sf::RenderWindow& window, Camera& camera, const Transform& transform, Shape* shape, const sf::Color& color)
	{
		assert(shape->type() == Shape::Type::Edge);
		Edge* edge = static_cast<Edge*>(shape);
		renderPoint(window, camera, edge->startPoint() + transform.position, color);
		renderPoint(window, camera, edge->endPoint() + transform.position, color);
		renderLine(window, camera, edge->startPoint() + transform.position, edge->endPoint() + transform.position, color);

		Vec2 center = (edge->startPoint() + edge->endPoint()) / 2.0f;
		center += transform.position;
		renderLine(window, camera, center, center + 0.1f * edge->normal(), RenderConstant::MaterialYellow);
	}
	void RenderSFMLImpl::renderRectangle(sf::RenderWindow& window, Camera& camera, const Transform& transform, Shape* shape, const sf::Color& color)
	{
		assert(shape != nullptr);
		assert(shape->type() == Shape::Type::Polygon);
		renderPolygon(window, camera, transform, shape, color);
	}
	void RenderSFMLImpl::renderCircle(sf::RenderWindow& window, Camera& camera, const Transform& transform, Shape* shape, const sf::Color& color)
	{
		assert(shape->type() == Shape::Type::Circle);
		const Circle* circle = static_cast<Circle*>(shape);
		const Vec2 screenPos = camera.worldToScreen(transform.position);
		sf::CircleShape circleShape(circle->radius() * RenderConstant::ScaleFactor * camera.meterToPixel());
		sf::Color fillColor(color);
		fillColor.a = RenderConstant::FillAlpha;
		circleShape.move(toVec2f(screenPos) - sf::Vector2f(circleShape.getRadius(), circleShape.getRadius()));
		circleShape.setFillColor(fillColor);
		circleShape.setOutlineThickness(RenderConstant::BorderSize);
		circleShape.setOutlineColor(color);
		circleShape.setPointCount(RenderConstant::BasicCirclePointCount + size_t(camera.meterToPixel()));
		window.draw(circleShape);
	}
	void RenderSFMLImpl::renderCapsule(sf::RenderWindow& window, Camera& camera, const Transform& transform, Shape* shape, const sf::Color& color)
	{
		assert(shape->type() == Shape::Type::Capsule);
		Container::Vector<sf::Vertex> vertices;

		const Capsule* capsule = static_cast<Capsule*>(shape);
		const Vec2 screenPos = camera.worldToScreen(transform.position);
		int pointCounts = (RenderConstant::BasicCirclePointCount + camera.meterToPixel()) / 4;
		sf::Vertex centerVertex = toVec2f(screenPos);
		sf::Color fillColor(color);
		fillColor.a = RenderConstant::FillAlpha;
		centerVertex.color = fillColor;
		vertices.emplace_back(centerVertex);
		auto sampling = [&](const Vec2& center, const real& radius, const real& startRadians, const real& endRadians)
		{
			real step = (endRadians - startRadians) / float(pointCounts);
			for (real radian = startRadians; radian <= endRadians; radian += step)
			{
				Vec2 point(radius * cosx(radian), radius * sinx(radian));
				point += center;
				const Vec2 worldPos = transform.transform(point * RenderConstant::ScaleFactor);
				const Vec2 screenPos = camera.worldToScreen(worldPos);
				sf::Vertex vertex;
				vertex.position = toVec2f(screenPos);
				vertex.color = fillColor;
				vertices.emplace_back(vertex);
			}
		};
		if (capsule->width() > capsule->height())
		{
			real radius = capsule->height() / 2.0f;
			sampling((capsule->bottomLeft() + capsule->topLeft()) / 2.0f, radius, degreeToRadian(90), degreeToRadian(270));
			sampling((capsule->topRight() + capsule->bottomRight()) / 2.0f, radius, degreeToRadian(270), degreeToRadian(450));
		}
		else
		{
			real radius = capsule->width() / 2.0f;
			sampling((capsule->topLeft() + capsule->topRight()) / 2.0f, radius, degreeToRadian(0), degreeToRadian(180));
			sampling((capsule->bottomLeft() + capsule->bottomRight()) / 2.0f, radius, degreeToRadian(180), degreeToRadian(360));
		}
		vertices.emplace_back(vertices[1]);
		window.draw(&vertices[0], vertices.size(), sf::TriangleFan);
		for (auto& elem : vertices)
			elem.color = color;
		window.draw(&vertices[1], vertices.size() - 1, sf::LinesStrip);
	}
	void RenderSFMLImpl::renderSector(sf::RenderWindow& window, Camera& camera, const Transform& transform, Shape* shape, const sf::Color& color)
	{
		assert(shape->type() == Shape::Type::Sector);
		Container::Vector<sf::Vertex> vertices;

	}
	void RenderSFMLImpl::renderEllipse(sf::RenderWindow& window, Camera& camera, const Transform& transform, Shape* shape, const sf::Color& color)
	{
		assert(shape->type() == Shape::Type::Ellipse);
		Container::Vector<sf::Vertex> vertices;

		const Ellipse* ellipse = static_cast<Ellipse*>(shape);
		const Vec2 screenPos = camera.worldToScreen(transform.position);
		int pointCounts = (RenderConstant::BasicCirclePointCount + camera.meterToPixel()) / 2;

		sf::Vertex centerVertex = toVec2f(screenPos);
		sf::Color fillColor(color);
		fillColor.a = RenderConstant::FillAlpha;
		centerVertex.color = fillColor;
		vertices.emplace_back(centerVertex);

		real step = Constant::TwoPi / float(pointCounts);
		real innerRadius, outerRadius;
		innerRadius = ellipse->A();
		outerRadius = ellipse->B();
		if (ellipse->A() > ellipse->B())
		{
			innerRadius = ellipse->B();
			outerRadius = ellipse->A();
		}
		for (real radian = 0; radian <= Constant::TwoPi; radian += step)
		{
			Vec2 point(outerRadius * cosx(radian), innerRadius * sinx(radian));

			const Vec2 worldPos = transform.transform(point * RenderConstant::ScaleFactor);
			const Vec2 screenPos = camera.worldToScreen(worldPos);
			sf::Vertex vertex;
			vertex.position = toVec2f(screenPos);
			vertex.color = fillColor;
			vertices.emplace_back(vertex);
		}
		vertices.emplace_back(vertices[1]);
		window.draw(&vertices[0], vertices.size(), sf::TriangleFan);
		for (auto& elem : vertices)
			elem.color = color;
		window.draw(&vertices[1], vertices.size() - 1, sf::LinesStrip);
	}
	void RenderSFMLImpl::renderCurve(sf::RenderWindow& window, Camera& camera, const Transform& transform, Shape* shape, const sf::Color& color)
	{
		assert(shape->type() == Shape::Type::Curve);
		Container::Vector<sf::Vertex> vertices;
		
	}
	void RenderSFMLImpl::renderAngleLine(sf::RenderWindow& window, Camera& camera, const Transform& transform, Shape* shape, const sf::Color& color)
	{
		sf::Color colorX(139, 195, 74);
		sf::Color colorY(255, 235, 59);
		colorX.a = 204;
		colorY.a = 204;
		Vec2 xP(0.1f, 0);
		Vec2 yP(0, 0.1f);
		Vec2 mc = transform.rotate(shape->center());
		xP = transform.transform(xP) + mc;
		yP = transform.transform(yP) + mc;
		renderLine(window, camera, transform.position + mc, xP, colorX);
		renderLine(window, camera, transform.position + mc, yP, colorY);
	}
	void RenderSFMLImpl::renderBody(sf::RenderWindow& window, Camera& camera, Body* body, const sf::Color& color)
	{
		Transform transform;
		transform.rotation = body->rotation();
		transform.position = body->position();
		renderShape(window, camera, transform, body->shape(), color);
	}
	void RenderSFMLImpl::renderAABB(sf::RenderWindow& window, Camera& camera, const AABB& aabb, const sf::Color& color)
	{
		Vec2 aabbSize(aabb.width, aabb.height);
		aabbSize *= camera.meterToPixel();
		sf::RectangleShape shape(toVec2f(aabbSize));
		shape.move(toVec2f(camera.worldToScreen(aabb.topLeft())));
		shape.setFillColor(sf::Color::Transparent);
		shape.setOutlineThickness(RenderConstant::BorderSize);
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
		DistanceJoint* distanceJoint = static_cast<DistanceJoint*>(joint);
		Vec2 pa = distanceJoint->primitive().bodyA->toWorldPoint(distanceJoint->primitive().localPointA);
		Vec2 pb = distanceJoint->primitive().targetPoint;
		Vec2 n = (pa - pb).normal();
		Vec2 minPoint = n * distanceJoint->primitive().minDistance + pb;
		Vec2 maxPoint = n * distanceJoint->primitive().maxDistance + pb;
		sf::Color minColor = RenderConstant::MaterialBlue;
		sf::Color maxColor = RenderConstant::MaterialRed;
		minColor.a = 204;
		maxColor.a = 204;
		renderPoint(window, camera, pa, RenderConstant::MaterialGray);
		renderPoint(window, camera, pb, RenderConstant::MaterialGray);
		renderPoint(window, camera, minPoint, minColor);
		renderPoint(window, camera, maxPoint, maxColor);
		sf::Color lineColor = RenderConstant::MaterialGray;
		lineColor.a = 150;
		renderLine(window, camera, pa, pb, lineColor);

	}
	void RenderSFMLImpl::renderPointJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color)
	{
		assert(joint != nullptr);
		PointJoint* pointJoint = static_cast<PointJoint*>(joint);
		Vec2 pa = pointJoint->primitive().bodyA->toWorldPoint(pointJoint->primitive().localPointA);
		Vec2 pb = pointJoint->primitive().targetPoint;

		sf::Color point = RenderConstant::MaterialOrange;
		sf::Color green = sf::Color::Green;
		point.a = 204;
		green.a = 78;

		renderPoint(window, camera, pa, point, 2);
		renderPoint(window, camera, pb, point, 2);
		renderLine(window, camera, pa, pb, green);
	}
	void RenderSFMLImpl::renderOrientationJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color)
	{
		assert(joint != nullptr);
		OrientationJoint* pointJoint = static_cast<OrientationJoint*>(joint);
		Vec2 pa = pointJoint->primitive().bodyA->position();
		Vec2 pb = pointJoint->primitive().targetPoint;

		sf::Color point = RenderConstant::MaterialOrange;
		sf::Color green = sf::Color::Green;
		point.a = 204;
		green.a = 78;

		renderPoint(window, camera, pa, point, 2);
		renderPoint(window, camera, pb, point, 2);
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
		RevoluteJoint* revoluteJoint = static_cast<RevoluteJoint*>(joint);
		Vec2 pa = revoluteJoint->primitive().bodyA->toWorldPoint(revoluteJoint->primitive().localPointA);
		Vec2 pb = revoluteJoint->primitive().bodyB->toWorldPoint(revoluteJoint->primitive().localPointB);

		sf::Color point = RenderConstant::MaterialOrange;
		sf::Color green = sf::Color::Green;
		point.a = 204;
		green.a = 78;

		renderPoint(window, camera, pa, point, 2);
		renderPoint(window, camera, pb, point, 2);
		renderLine(window, camera, pa, pb, green);
	}
	void RenderSFMLImpl::renderWheelJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color)
	{
	}
}