#include "render.h"
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
		shape.move(toVector2f(camera.worldToScreen(point)) - sf::Vector2f(RenderConstant::PointSize, RenderConstant::PointSize));
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
	void RenderSFMLImpl::renderPoints(sf::RenderWindow& window, Camera& camera, const Container::Vector<Vector2>& points, const sf::Color& color)
	{
		Container::Vector<sf::Vertex> vertices;
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



	void RenderSFMLImpl::renderLines(sf::RenderWindow& window, Camera& camera, const Container::Vector<std::pair<Vector2, Vector2>>& lines, const sf::Color& color)
	{
		Container::Vector<sf::Vertex> vertices;
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
		default:
			break;
		}
	}
	void RenderSFMLImpl::renderPolygon(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color)
	{
		assert(shape.shape != nullptr);
		assert(shape.shape->type() == Shape::Type::Polygon);
		sf::ConvexShape convex;
		Polygon* polygon = static_cast<Polygon*>(shape.shape);
		convex.setPointCount(polygon->vertices().size());
		for (size_t i = 0; i < polygon->vertices().size(); ++i)
		{
			const Vector2 worldPos = shape.transform.translatePoint(polygon->vertices()[i] * RenderConstant::ScaleFactor);
			const Vector2 screenPos = camera.worldToScreen(worldPos);
			convex.setPoint(i, toVector2f(screenPos));
		}
		sf::Color fillColor(color);
		fillColor.a = RenderConstant::FillAlpha;
		convex.setFillColor(fillColor);
		convex.setOutlineThickness(RenderConstant::BorderSize);
		convex.setOutlineColor(color);
		window.draw(convex);
	}
	void RenderSFMLImpl::renderEdge(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color)
	{
		assert(shape.shape->type() == Shape::Type::Edge);
		Edge* edge = static_cast<Edge*>(shape.shape);
		renderPoint(window, camera, edge->startPoint() + shape.transform.position, color);
		renderPoint(window, camera, edge->endPoint() + shape.transform.position, color);
		renderLine(window, camera, edge->startPoint() + shape.transform.position, edge->endPoint() + shape.transform.position, color);

		Vector2 center = (edge->startPoint() + edge->endPoint()) / 2.0f;
		center += shape.transform.position;
		renderLine(window, camera, center, center + 0.1f * edge->normal(), RenderConstant::MaterialYellow);
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
		const Circle* circle = static_cast<Circle*>(shape.shape);
		const Vector2 screenPos = camera.worldToScreen(shape.transform.position);
		sf::CircleShape circleShape(circle->radius() * RenderConstant::ScaleFactor * camera.meterToPixel());
		sf::Color fillColor(color);
		fillColor.a = RenderConstant::FillAlpha;
		circleShape.move(toVector2f(screenPos) - sf::Vector2f(circleShape.getRadius(), circleShape.getRadius()));
		circleShape.setFillColor(fillColor);
		circleShape.setOutlineThickness(RenderConstant::BorderSize);
		circleShape.setOutlineColor(color);
		circleShape.setPointCount(RenderConstant::BasicCirclePointCount + size_t(camera.meterToPixel()));
		window.draw(circleShape);
	}
	void RenderSFMLImpl::renderCapsule(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color)
	{
		assert(shape.shape->type() == Shape::Type::Capsule);
		Container::Vector<sf::Vertex> vertices;

		const Capsule* capsule = static_cast<Capsule*>(shape.shape);
		const Vector2 screenPos = camera.worldToScreen(shape.transform.position);
		int pointCounts = (RenderConstant::BasicCirclePointCount + camera.meterToPixel()) / 4;
		sf::Vertex centerVertex = toVector2f(screenPos);
		sf::Color fillColor(color);
		fillColor.a = RenderConstant::FillAlpha;
		centerVertex.color = fillColor;
		vertices.emplace_back(centerVertex);
		auto sampling = [&](const Vector2& center, const real& radius, const real& startRadians, const real& endRadians)
		{
			real step = (endRadians - startRadians) / float(pointCounts);
			for (real radian = startRadians; radian <= endRadians; radian += step)
			{
				Vector2 point(radius * Math::cosx(radian), radius * Math::sinx(radian));
				point += center;
				const Vector2 worldPos = Matrix2x2(shape.transform.rotation).multiply(point * RenderConstant::ScaleFactor) + shape.transform.position;
				const Vector2 screenP = camera.worldToScreen(worldPos);
				sf::Vertex vertex;
				vertex.position = toVector2f(screenP);
				vertex.color = fillColor;
				vertices.emplace_back(vertex);
			}
		};
		if (capsule->halfWidth() > capsule->halfHeight())
		{
			real radius = capsule->halfHeight();
			sampling((capsule->bottomLeft() + capsule->topLeft()) / 2.0f, radius, Math::degreeToRadian(90), Math::degreeToRadian(270));
			sampling((capsule->topRight() + capsule->bottomRight()) / 2.0f, radius, Math::degreeToRadian(270), Math::degreeToRadian(450));
		}
		else
		{
			real radius = capsule->halfWidth();
			sampling((capsule->topLeft() + capsule->topRight()) / 2.0f, radius, Math::degreeToRadian(0), Math::degreeToRadian(180));
			sampling((capsule->bottomLeft() + capsule->bottomRight()) / 2.0f, radius, Math::degreeToRadian(180), Math::degreeToRadian(360));
		}
		vertices.emplace_back(vertices[1]);
		window.draw(&vertices[0], vertices.size(), sf::TriangleFan);
		for (auto& elem : vertices)
			elem.color = color;
		window.draw(&vertices[1], vertices.size() - 1, sf::LinesStrip);
	}
	void RenderSFMLImpl::renderEllipse(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color)
	{
		assert(shape.shape->type() == Shape::Type::Ellipse);
		Container::Vector<sf::Vertex> vertices;

		const Ellipse* ellipse = static_cast<Ellipse*>(shape.shape);
		const Vector2 screenPos = camera.worldToScreen(shape.transform.position);
		int pointCounts = (RenderConstant::BasicCirclePointCount + camera.meterToPixel()) / 2;

		sf::Vertex centerVertex = toVector2f(screenPos);
		sf::Color fillColor(color);
		fillColor.a = RenderConstant::FillAlpha;
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
			const Vector2 worldPos = shape.transform.translatePoint(point * RenderConstant::ScaleFactor);
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
	void RenderSFMLImpl::renderAngleLine(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape, const sf::Color& color)
	{
		sf::Color colorX(139, 195, 74);
		sf::Color colorY(255, 235, 59);
		colorX.a = 204;
		colorY.a = 204;
		Vector2 xP(0.1f, 0);
		Vector2 yP(0, 0.1f);
		xP = shape.transform.translatePoint(xP);
		yP = shape.transform.translatePoint(yP);
		renderLine(window, camera, shape.transform.position, xP, colorX);
		renderLine(window, camera, shape.transform.position, yP, colorY);
	}
	void RenderSFMLImpl::renderBody(sf::RenderWindow& window, Camera& camera, Body* body, const sf::Color& color)
	{
		ShapePrimitive primitive;
		primitive.shape = body->shape();
		primitive.transform.rotation = body->rotation();
		primitive.transform.position = body->position();
		renderShape(window, camera, primitive, color);
	}
	void RenderSFMLImpl::renderAABB(sf::RenderWindow& window, Camera& camera, const AABB& aabb, const sf::Color& color)
	{
		Vector2 aabbSize(aabb.width, aabb.height);
		aabbSize *= camera.meterToPixel();
		sf::RectangleShape shape(toVector2f(aabbSize));
		shape.move(toVector2f(camera.worldToScreen(aabb.topLeft())));
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
		Vector2 pa = distanceJoint->primitive().bodyA->toWorldPoint(distanceJoint->primitive().localPointA);
		Vector2 pb = distanceJoint->primitive().targetPoint;
		Vector2 n = (pa - pb).normal();
		Vector2 minPoint = n * distanceJoint->primitive().minDistance + pb;
		Vector2 maxPoint = n * distanceJoint->primitive().maxDistance + pb;
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
		Vector2 pa = pointJoint->primitive().bodyA->toWorldPoint(pointJoint->primitive().localPointA);
		Vector2 pb = pointJoint->primitive().targetPoint;

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
		Vector2 pa = pointJoint->primitive().bodyA->position();
		Vector2 pb = pointJoint->primitive().targetPoint;

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
		Vector2 pa = revoluteJoint->primitive().bodyA->toWorldPoint(revoluteJoint->primitive().localPointA);
		Vector2 pb = revoluteJoint->primitive().bodyB->toWorldPoint(revoluteJoint->primitive().localPointB);

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

	void RenderSFMLImpl::renderSimplex(sf::RenderWindow& window, Camera& camera, const Simplex& simplex,
		const sf::Color& color)
	{
		sf::Color lineColor = color;
		lineColor.a = 150;
		switch (simplex.count)
		{
		case 0:
			break;
		case 1:
			renderPoint(window, camera, simplex.vertices[0].result, color);
			break;
		case 2:
			renderLine(window, camera, simplex.vertices[0].result, simplex.vertices[1].result, lineColor);
			renderPoint(window, camera, simplex.vertices[0].result, color);
			renderPoint(window, camera, simplex.vertices[1].result, color);
			break;
		case 3:
			renderLine(window, camera, simplex.vertices[0].result, simplex.vertices[1].result, lineColor);
			renderLine(window, camera, simplex.vertices[1].result, simplex.vertices[2].result, lineColor);
			renderLine(window, camera, simplex.vertices[2].result, simplex.vertices[0].result, lineColor);
			renderPoint(window, camera, simplex.vertices[0].result, color);
			renderPoint(window, camera, simplex.vertices[1].result, color);
			renderPoint(window, camera, simplex.vertices[2].result, color);
			break;
		default:
			assert(false && "Simplex count is more than 3");
			break;
		}
	}

	void RenderSFMLImpl::renderArrow(sf::RenderWindow& window, Camera& camera, const Vector2& start, const Vector2& end,
		const sf::Color& color, const real& size, const real& degree)
	{
		renderLine(window, camera, start, end, color);
		Vector2 tf = start - end;
		real length = tf.length();
		real scale = size;
		if (length < 1.0f)
			scale = length * size;
		if (realEqual(length, 0))
			return;
		Vector2 normal = tf / length * scale;
		Matrix2x2 mat(Math::degreeToRadian(degree));
		Vector2 p1 = mat.multiply(normal) + end;
		mat.set(Math::degreeToRadian(-degree));
		Vector2 p2 = mat.multiply(normal) + end;

		Vector2 v1 = camera.worldToScreen(end);
		Vector2 v2 = camera.worldToScreen(p1);
		Vector2 v3 = camera.worldToScreen(p2);

		sf::ConvexShape convex;
		convex.setPointCount(3);
		convex.setPoint(0, toVector2f(v1));
		convex.setPoint(1, toVector2f(v2));
		convex.setPoint(2, toVector2f(v3));
		
		convex.setFillColor(color);
		convex.setOutlineThickness(RenderConstant::BorderSize);
		convex.setOutlineColor(color);
		window.draw(convex);
	}
}
