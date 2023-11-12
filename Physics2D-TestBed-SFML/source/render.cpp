#include "render.h"

namespace Physics2D
{
	sf::Vector2f RenderSFMLImpl::toVector2f(const Vector2& vector)
	{
		return sf::Vector2f(vector.x, vector.y);
	}

	void RenderSFMLImpl::renderPoint(sf::RenderWindow& window, Camera& camera, const Vector2& point,
	                                 const sf::Color& color, const real pointSize)
	{
		sf::CircleShape shape(pointSize);
		shape.setFillColor(color);
		shape.move(toVector2f(camera.worldToScreen(point)) - sf::Vector2f(pointSize, pointSize));
		window.draw(shape);
	}

	void RenderSFMLImpl::renderLine(sf::RenderWindow& window, Camera& camera, const Vector2& p1, const Vector2& p2,
	                                const sf::Color& color)
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

	void RenderSFMLImpl::renderPoints(sf::RenderWindow& window, Camera& camera,
	                                  const Container::Vector<Vector2>& points, const sf::Color& color)
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


	void RenderSFMLImpl::renderLines(sf::RenderWindow& window, Camera& camera,
	                                 const Container::Vector<std::pair<Vector2, Vector2>>& lines,
	                                 const sf::Color& color)
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


	void RenderSFMLImpl::renderShape(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape,
	                                 const sf::Color& color)
	{
		switch (shape.shape->type())
		{
		case ShapeType::Polygon:
			{
				renderPolygon(window, camera, shape, color);
				break;
			}
		case ShapeType::Ellipse:
			{
				renderEllipse(window, camera, shape, color);
				break;
			}
		case ShapeType::Circle:
			{
				renderCircle(window, camera, shape, color);
				break;
			}
		case ShapeType::Edge:
			{
				renderEdge(window, camera, shape, color);
				break;
			}
		case ShapeType::Capsule:
			{
				renderCapsule(window, camera, shape, color);
				break;
			}
		default:
			break;
		}
	}

	void RenderSFMLImpl::renderPolygon(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape,
	                                   const sf::Color& color)
	{
		assert(shape.shape != nullptr);
		assert(shape.shape->type() == ShapeType::Polygon);
		sf::ConvexShape convex;
		auto polygon = static_cast<Polygon*>(shape.shape);
		convex.setPointCount(polygon->vertices().size());
		for (size_t i = 0; i < polygon->vertices().size(); ++i)
		{
			const Vector2 worldPos = shape.transform.translatePoint(
				polygon->vertices()[i] * RenderConstant::ScaleFactor);
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

	void RenderSFMLImpl::renderEdge(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape,
	                                const sf::Color& color)
	{
		assert(shape.shape->type() == ShapeType::Edge);
		auto edge = static_cast<Edge*>(shape.shape);
		renderPoint(window, camera, edge->startPoint() + shape.transform.position, color);
		renderPoint(window, camera, edge->endPoint() + shape.transform.position, color);
		renderLine(window, camera, edge->startPoint() + shape.transform.position,
		           edge->endPoint() + shape.transform.position, color);

		Vector2 center = (edge->startPoint() + edge->endPoint()) / 2.0f;
		center += shape.transform.position;
		renderLine(window, camera, center, center + 0.1f * edge->normal(), RenderConstant::Yellow);
	}

	void RenderSFMLImpl::renderRectangle(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape,
	                                     const sf::Color& color)
	{
		assert(shape.shape != nullptr);
		assert(shape.shape->type() == ShapeType::Polygon);
		renderPolygon(window, camera, shape, color);
	}

	void RenderSFMLImpl::renderCircle(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape,
	                                  const sf::Color& color)
	{
		assert(shape.shape->type() == ShapeType::Circle);
		const Circle* circle = static_cast<Circle*>(shape.shape);
		const Vector2 screenPos = camera.worldToScreen(shape.transform.position);
		sf::CircleShape circleShape(circle->radius() * RenderConstant::ScaleFactor * camera.meterToPixel());
		sf::Color fillColor(color);
		fillColor.a = RenderConstant::FillAlpha;
		circleShape.move(toVector2f(screenPos) - sf::Vector2f(circleShape.getRadius(), circleShape.getRadius()));
		circleShape.setFillColor(fillColor);
		circleShape.setOutlineThickness(RenderConstant::BorderSize);
		circleShape.setOutlineColor(color);
		circleShape.setPointCount(RenderConstant::BasicCirclePointCount + static_cast<size_t>(camera.meterToPixel()));
		window.draw(circleShape);
	}

	void RenderSFMLImpl::renderCapsule(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape,
	                                   const sf::Color& color)
	{
		assert(shape.shape->type() == ShapeType::Capsule);
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
			real step = (endRadians - startRadians) / static_cast<float>(pointCounts);
			for (real radian = startRadians; radian <= endRadians; radian += step)
			{
				Vector2 point(radius * Math::cosx(radian), radius * Math::sinx(radian));
				point += center;
				const Vector2 worldPos = Matrix2x2(shape.transform.rotation).multiply(
					point * RenderConstant::ScaleFactor) + shape.transform.position;
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
			sampling((capsule->bottomLeft() + capsule->topLeft()) / 2.0f, radius, Math::degreeToRadian(90),
			         Math::degreeToRadian(270));
			sampling((capsule->topRight() + capsule->bottomRight()) / 2.0f, radius, Math::degreeToRadian(270),
			         Math::degreeToRadian(450));
		}
		else
		{
			real radius = capsule->halfWidth();
			sampling((capsule->topLeft() + capsule->topRight()) / 2.0f, radius, Math::degreeToRadian(0),
			         Math::degreeToRadian(180));
			sampling((capsule->bottomLeft() + capsule->bottomRight()) / 2.0f, radius, Math::degreeToRadian(180),
			         Math::degreeToRadian(360));
		}
		vertices.emplace_back(vertices[1]);
		window.draw(&vertices[0], vertices.size(), sf::TriangleFan);
		for (auto& elem : vertices)
			elem.color = color;
		window.draw(&vertices[1], vertices.size() - 1, sf::LinesStrip);
	}

	void RenderSFMLImpl::renderEllipse(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape,
	                                   const sf::Color& color)
	{
		assert(shape.shape->type() == ShapeType::Ellipse);
		Container::Vector<sf::Vertex> vertices;

		const Ellipse* ellipse = static_cast<Ellipse*>(shape.shape);
		const Vector2 screenPos = camera.worldToScreen(shape.transform.position);
		int pointCounts = (RenderConstant::BasicCirclePointCount + camera.meterToPixel()) / 2;

		sf::Vertex centerVertex = toVector2f(screenPos);
		sf::Color fillColor(color);
		fillColor.a = RenderConstant::FillAlpha;
		centerVertex.color = fillColor;
		vertices.emplace_back(centerVertex);

		real step = Constant::DoublePi / static_cast<float>(pointCounts);
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

	void RenderSFMLImpl::renderAngleLine(sf::RenderWindow& window, Camera& camera, const ShapePrimitive& shape,
	                                     const sf::Color& color)
	{
		sf::Color colorX(3, 169, 244);
		sf::Color colorY(244, 67, 54);
		Vector2 xP(0.15f, 0);
		Vector2 yP(0, 0.15f);
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
		case JointType::Weld:
		{
			renderWeldJoint(window, camera, joint, color);
			break;
		}
		case JointType::Motor:
		{
			renderMotorJoint(window, camera, joint, color);
			break;
		}
		case JointType::Path:
		{
			renderPathJoint(window, camera, joint, color);
			break;
		}
		default:
			break;
		}
	}

	void RenderSFMLImpl::renderRotationJoint(sf::RenderWindow& window, Camera& camera, Joint* joint,
	                                         const sf::Color& color)
	{
	}

	void RenderSFMLImpl::renderDistanceJoint(sf::RenderWindow& window, Camera& camera, Joint* joint,
	                                         const sf::Color& color)
	{
		assert(joint != nullptr);
		auto distanceJoint = static_cast<DistanceJoint*>(joint);
		Vector2 pa = distanceJoint->primitive().bodyA->toWorldPoint(distanceJoint->primitive().localPointA);
		Vector2 pb = distanceJoint->primitive().bodyB->toWorldPoint(distanceJoint->primitive().localPointB);
		Vector2 n = (pa - pb).normal();
		Vector2 mid = pb + n * (pa - pb).length() * 0.5f;

		Vector2 maxPoint1 = mid + 0.5f * n * distanceJoint->primitive().maxDistance;
		Vector2 maxPoint2 = mid - 0.5f * n * distanceJoint->primitive().maxDistance;
		Vector2 minPoint1 = mid + 0.5f * n * distanceJoint->primitive().minDistance;
		Vector2 minPoint2 = mid - 0.5f * n * distanceJoint->primitive().minDistance;
		
		sf::Color minColor = RenderConstant::Blue;
		sf::Color maxColor = RenderConstant::Red;
		minColor.a = 204;
		maxColor.a = 204;
		renderPoint(window, camera, pa, RenderConstant::Gray);
		renderPoint(window, camera, pb, RenderConstant::Gray);
		renderPoint(window, camera, minPoint1, minColor);
		renderPoint(window, camera, maxPoint1, maxColor);
		renderPoint(window, camera, minPoint2, minColor);
		renderPoint(window, camera, maxPoint2, maxColor);
		sf::Color lineColor = RenderConstant::DarkGreen;
		lineColor.a = 150;
		renderLine(window, camera, pa, pb, lineColor);
	}

	void renderMotorJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color)
	{
		assert(joint != nullptr);
	}
	void RenderSFMLImpl::renderPointJoint(sf::RenderWindow& window, Camera& camera, Joint* joint,
	                                      const sf::Color& color)
	{
		assert(joint != nullptr);
		auto pointJoint = static_cast<PointJoint*>(joint);
		Vector2 pa = pointJoint->primitive().bodyA->toWorldPoint(pointJoint->primitive().localPointA);
		Vector2 pb = pointJoint->primitive().targetPoint;

		sf::Color point = RenderConstant::Orange;
		sf::Color green = sf::Color::Green;
		point.a = 204;
		green.a = 78;

		renderPoint(window, camera, pa, point, 2);
		renderPoint(window, camera, pb, point, 2);
		renderLine(window, camera, pa, pb, green);
	}

	void RenderSFMLImpl::renderMotorJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color)
	{
		assert(joint != nullptr);
		auto motorJoint = static_cast<MotorJoint*>(joint);
		//Vector2 pa = motorJoint->primitive().bodyA->toWorldPoint(motorJoint->primitive().localPointA);
		//Vector2 pb = motorJoint->primitive().bodyB->toWorldPoint(motorJoint->primitive().localPointB);

		//sf::Color point = RenderConstant::Pink;
		//sf::Color green = sf::Color::Green;
		//point.a = 204;
		//green.a = 78;

		//renderLine(window, camera, pa, pb, green);

		Vector2 pa = motorJoint->primitive().bodyA->toWorldPoint(motorJoint->primitive().localPointA);
		Vector2 pb = motorJoint->primitive().bodyB->toWorldPoint(motorJoint->primitive().localPointB);
		sf::Color point = RenderConstant::Orange;
		sf::Color green = sf::Color::Green;
		point.a = 204;
		green.a = 78;
		renderPoint(window, camera, pa, point, 2);
		renderPoint(window, camera, pb, point, 2);
		renderLine(window, camera, pa, pb, green);
		sf::Color colorX(3, 169, 244);
		sf::Color colorY(244, 67, 54);
		Vector2 xP(0.15f, 0);
		Vector2 yP(0, 0.15f);


		Vector2 s = motorJoint->primitive().bodyA->toWorldPoint(motorJoint->primitive().localPointA);
		xP = motorJoint->primitive().bodyA->toWorldPoint(motorJoint->primitive().localPointA + xP);
		yP = motorJoint->primitive().bodyA->toWorldPoint(motorJoint->primitive().localPointA + yP);
		renderLine(window, camera, s, xP, colorX);
		renderLine(window, camera, s, yP, colorY);

		colorX = sf::Color(255, 235, 59);
		colorY = sf::Color(139, 195, 74);


		xP.set(0.15f, 0);
		yP.set(0, 0.15f);

		s = motorJoint->primitive().bodyB->toWorldPoint(motorJoint->primitive().localPointB);
		xP = motorJoint->primitive().bodyB->toWorldPoint(motorJoint->primitive().localPointB + xP);
		yP = motorJoint->primitive().bodyB->toWorldPoint(motorJoint->primitive().localPointB + yP);
		renderLine(window, camera, s, xP, colorX);
		renderLine(window, camera, s, yP, colorY);


		renderPoint(window, camera, pa, RenderConstant::Red, 2);
		renderPoint(window, camera, pb, RenderConstant::Blue, 2);
	}

	void RenderSFMLImpl::renderPathJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color)
	{
		assert(joint != nullptr);
		auto pathJoint = static_cast<PathJoint*>(joint);

		Vector2 pa = pathJoint->primitive().bodyA->toWorldPoint(pathJoint->primitive().localPointA);
		Vector2 origin = pathJoint->primitive().origin;
		const Vector2 screenPos = camera.worldToScreen(origin);
		sf::CircleShape circleShape(pathJoint->primitive().radius * RenderConstant::ScaleFactor * camera.meterToPixel());
		sf::Color fillColor(RenderConstant::Gray);
		fillColor.a = 0;
		circleShape.move(toVector2f(screenPos) - sf::Vector2f(circleShape.getRadius(), circleShape.getRadius()));
		circleShape.setFillColor(fillColor);
		circleShape.setOutlineThickness(RenderConstant::BorderSize);
		fillColor.a = 80;
		circleShape.setOutlineColor(fillColor);
		circleShape.setPointCount(RenderConstant::BasicCirclePointCount + static_cast<size_t>(camera.meterToPixel()));
		window.draw(circleShape);

		renderPoint(window, camera, pa, RenderConstant::Red, 2);
	}

	void RenderSFMLImpl::renderDashedLine(sf::RenderWindow& window, Camera& camera, const Vector2& p1, const Vector2& p2, 
	                                      const sf::Color& color, const real& dashLength, const real& dashGap)
	{
		sf::VertexArray lines(sf::Lines);

		Vector2 direction = p2 - p1;
		real length = direction.length();
		direction /= length;
		for(real d = 0; d < length; d += dashLength + dashGap)
		{
			Vector2 dashStart = p1 + direction * d;
			Vector2 dashEnd = p1 + direction * Math::min(d + dashLength, direction.length());
			lines.append(sf::Vertex(toVector2f(camera.worldToScreen(dashStart)), color));
			lines.append(sf::Vertex(toVector2f(camera.worldToScreen(dashEnd)), color));
		}

		window.draw(lines);
	}

	void RenderSFMLImpl::renderOrientationJoint(sf::RenderWindow& window, Camera& camera, Joint* joint,
	                                            const sf::Color& color)
	{
		assert(joint != nullptr);
		auto pointJoint = static_cast<OrientationJoint*>(joint);
		Vector2 pa = pointJoint->primitive().bodyA->position();
		Vector2 pb = pointJoint->primitive().targetPoint;

		sf::Color point = RenderConstant::Orange;
		sf::Color green = sf::Color::Green;
		point.a = 204;
		green.a = 78;

		renderPoint(window, camera, pa, point, 2);
		renderPoint(window, camera, pb, point, 2);
		renderLine(window, camera, pa, pb, green);
	}

	void RenderSFMLImpl::renderPulleyJoint(sf::RenderWindow& window, Camera& camera, Joint* joint,
	                                       const sf::Color& color)
	{
	}

	void RenderSFMLImpl::renderPrismaticJoint(sf::RenderWindow& window, Camera& camera, Joint* joint,
	                                          const sf::Color& color)
	{
		assert(joint != nullptr);
		auto prismaticJoint = static_cast<PrismaticJoint*>(joint);
		Vector2 pa = prismaticJoint->primitive().bodyA->toWorldPoint(prismaticJoint->primitive().localPointA);
		Vector2 pb = prismaticJoint->primitive().bodyB->toWorldPoint(prismaticJoint->primitive().localPointB);
		Vector2 xAxis = prismaticJoint->primitive().xAxis;
		Vector2 yAxis = prismaticJoint->primitive().yAxis;

		renderPoint(window, camera, pa, RenderConstant::Red, 2);
		renderPoint(window, camera, pb, RenderConstant::Blue, 2);
		renderLine(window, camera, pa, pb, RenderConstant::LightCyan);

		bool infiniteLine = prismaticJoint->primitive().lowerLimit == prismaticJoint->primitive().upperLimit;
		if(!infiniteLine)
		{
			Vector2 lower = prismaticJoint->primitive().lowerLimit * xAxis;
			Vector2 upper = prismaticJoint->primitive().upperLimit * xAxis;
			renderLine(window, camera, pb + lower, pb + upper, RenderConstant::Gray);
			real shift = 0.25f;
			renderLine(window, camera, pb + lower + shift * yAxis, pb + lower - shift * yAxis, RenderConstant::LightBlue);
			renderLine(window, camera, pb + upper + shift * yAxis, pb + upper - shift * yAxis, RenderConstant::Pink);
		}

	}

	void RenderSFMLImpl::renderRevoluteJoint(sf::RenderWindow& window, Camera& camera, Joint* joint,
	                                         const sf::Color& color)
	{
		assert(joint != nullptr);
		auto revoluteJoint = static_cast<RevoluteJoint*>(joint);
		Vector2 pa = revoluteJoint->primitive().bodyA->toWorldPoint(revoluteJoint->primitive().localPointA);
		Vector2 pb = revoluteJoint->primitive().bodyB->toWorldPoint(revoluteJoint->primitive().localPointB);
		sf::Color point = RenderConstant::Orange;
		sf::Color green = sf::Color::Green;
		point.a = 204;
		green.a = 78;
		renderPoint(window, camera, pa, point, 2);
		renderPoint(window, camera, pb, point, 2);
		renderLine(window, camera, pa, pb, green);
		sf::Color colorX(3, 169, 244);
		sf::Color colorY(244, 67, 54);
		Vector2 xP(0.15f, 0);
		Vector2 yP(0, 0.15f);


		Vector2 s = revoluteJoint->primitive().bodyA->toWorldPoint(revoluteJoint->primitive().localPointA);
		xP = revoluteJoint->primitive().bodyA->toWorldPoint(revoluteJoint->primitive().localPointA + xP);
		yP = revoluteJoint->primitive().bodyA->toWorldPoint(revoluteJoint->primitive().localPointA + yP);
		renderLine(window, camera, s, xP, colorX);
		renderLine(window, camera, s, yP, colorY);

		colorX = sf::Color(255, 235, 59);
		colorY = sf::Color(139, 195, 74);


		xP.set(0.15f, 0);
		yP.set(0, 0.15f);

		s = revoluteJoint->primitive().bodyB->toWorldPoint(revoluteJoint->primitive().localPointB);
		xP = revoluteJoint->primitive().bodyB->toWorldPoint(revoluteJoint->primitive().localPointB + xP);
		yP = revoluteJoint->primitive().bodyB->toWorldPoint(revoluteJoint->primitive().localPointB + yP);
		renderLine(window, camera, s, xP, colorX);
		renderLine(window, camera, s, yP, colorY);


		renderPoint(window, camera, pa, RenderConstant::Red, 2);
		renderPoint(window, camera, pb, RenderConstant::Blue, 2);

		if(revoluteJoint->primitive().angularLimit)
		{
			real lower = revoluteJoint->primitive().lowerAngle + revoluteJoint->primitive().bodyB->rotation();
			real upper = revoluteJoint->primitive().upperAngle + revoluteJoint->primitive().bodyB->rotation();

			Vector2 axis(0.5f, 0.0f);
			Vector2 low = Matrix2x2(lower).multiply(axis);
			Vector2 up = Matrix2x2(upper).multiply(axis);

			renderDashedLine(window, camera, s, s + low, RenderConstant::LightCyan);
			renderDashedLine(window, camera, s, s + up, RenderConstant::Pink);

			
		}
	}

	void RenderSFMLImpl::renderWheelJoint(sf::RenderWindow& window, Camera& camera, Joint* joint,
	                                      const sf::Color& color)
	{
	}

	void RenderSFMLImpl::renderWeldJoint(sf::RenderWindow& window, Camera& camera, Joint* joint, const sf::Color& color)
	{
		assert(joint != nullptr);
		auto weldJoint = static_cast<WeldJoint*>(joint);
		Vector2 pa = weldJoint->primitive().bodyA->toWorldPoint(weldJoint->primitive().localPointA);
		Vector2 pb = weldJoint->primitive().bodyB->toWorldPoint(weldJoint->primitive().localPointB);
		sf::Color point = RenderConstant::Orange;
		sf::Color green = sf::Color::Green;
		point.a = 204;
		green.a = 78;
		renderPoint(window, camera, pa, point, 2);
		renderPoint(window, camera, pb, point, 2);
		renderLine(window, camera, pa, pb, green);
		sf::Color colorX(3, 169, 244);
		sf::Color colorY(244, 67, 54);
		Vector2 xP(0.15f, 0);
		Vector2 yP(0, 0.15f);
		

		Vector2 s = weldJoint->primitive().bodyA->toWorldPoint(weldJoint->primitive().localPointA);
		xP = weldJoint->primitive().bodyA->toWorldPoint(weldJoint->primitive().localPointA + xP);
		yP = weldJoint->primitive().bodyA->toWorldPoint(weldJoint->primitive().localPointA + yP);
		renderLine(window, camera, s, xP, colorX);
		renderLine(window, camera, s, yP, colorY);

		colorX = sf::Color(255, 235, 59);
		colorY = sf::Color(139, 195, 74);


		xP.set(0.15f, 0);
		yP.set(0, 0.15f);

		s = weldJoint->primitive().bodyB->toWorldPoint(weldJoint->primitive().localPointB);
		xP = weldJoint->primitive().bodyB->toWorldPoint(weldJoint->primitive().localPointB + xP);
		yP = weldJoint->primitive().bodyB->toWorldPoint(weldJoint->primitive().localPointB + yP);
		renderLine(window, camera, s, xP, colorX);
		renderLine(window, camera, s, yP, colorY);
	}

	void RenderSFMLImpl::renderSimplex(sf::RenderWindow& window, Camera& camera, const Simplex& simplex,
	                                   const sf::Color& color, const sf::Font& font, bool showIndex, const unsigned int& indexSize)
	{
		sf::Color lineColor = color;
		lineColor.a = 150;
		switch (simplex.count)
		{
		case 0:
			break;
		case 1:
			renderPoint(window, camera, simplex.vertices[0].result, color);
			if (showIndex)
				renderInt(window, camera, simplex.vertices[0].result, font, 0, lineColor, indexSize);
			break;
		case 2:
			renderLine(window, camera, simplex.vertices[0].result, simplex.vertices[1].result, lineColor);
			renderPoint(window, camera, simplex.vertices[0].result, color);
			renderPoint(window, camera, simplex.vertices[1].result, color);
			if (showIndex)
			{
				Vector2 offset = simplex.vertices[1].result - simplex.vertices[0].result;
				offset = -offset.perpendicular().normal() * 0.3f;
				renderInt(window, camera, simplex.vertices[0].result, font, 0, lineColor, indexSize, offset);
				renderInt(window, camera, simplex.vertices[1].result, font, 1, lineColor, indexSize, offset);
			}
			break;
		case 3:
			renderLine(window, camera, simplex.vertices[0].result, simplex.vertices[1].result, lineColor);
			renderLine(window, camera, simplex.vertices[1].result, simplex.vertices[2].result, lineColor);
			renderLine(window, camera, simplex.vertices[2].result, simplex.vertices[0].result, lineColor);
			renderPoint(window, camera, simplex.vertices[0].result, color);
			renderPoint(window, camera, simplex.vertices[1].result, color);
			renderPoint(window, camera, simplex.vertices[2].result, color);
			if (showIndex)
			{
				Vector2 center = (simplex.vertices[0].result + simplex.vertices[1].result + simplex.vertices[2].result)
					/ 3.0f;

				Vector2 offset = simplex.vertices[0].result - center;
				offset = offset.normal() * 0.3f;
				renderInt(window, camera, simplex.vertices[0].result, font, 0, lineColor, indexSize, offset);

				offset = simplex.vertices[1].result - center;
				offset = offset.normal() * 0.3f;
				renderInt(window, camera, simplex.vertices[1].result, font, 1, lineColor, indexSize, offset);

				offset = simplex.vertices[2].result - center;
				offset = offset.normal() * 0.3f;
				renderInt(window, camera, simplex.vertices[2].result, font, 2, lineColor, indexSize, offset);
			}
			break;
		default:
			assert(false && "Simplex count must be less than 3");
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

	void RenderSFMLImpl::renderText(sf::RenderWindow& window, Camera& camera, const Vector2& position,
	                                const sf::Font& font, const std::string& txt, const sf::Color& color,
	                                const unsigned int& size, const Vector2& screenOffset)
	{
		sf::Text text;
		text.setFont(font);
		text.setCharacterSize(size);
		text.setString(txt);
		text.setFillColor(color);
		sf::FloatRect text_rect = text.getLocalBounds();
		text.setOrigin(text_rect.left + text_rect.width / 2.0f, text_rect.top + text_rect.height / 2.0f);
		Vector2 offset = screenOffset;
		if (camera.meterToPixel() > camera.defaultMeterToPixel())
			offset /= camera.meterToPixel() / camera.defaultMeterToPixel();

		auto pos = toVector2f(camera.worldToScreen(position + offset));

		text.setPosition(pos);

		window.draw(text);
	}

	void RenderSFMLImpl::renderFloat(sf::RenderWindow& window, Camera& camera, const Vector2& position,
	                                 const sf::Font& font,
	                                 const real& value, const sf::Color& color, const unsigned int& size,
	                                 const Vector2& offset)
	{
		std::string str = std::format("{:.3f}", value);
		renderText(window, camera, position, font, str, color, size, offset);
	}

	void RenderSFMLImpl::renderInt(sf::RenderWindow& window, Camera& camera, const Vector2& position,
	                               const sf::Font& font, const int& value, const sf::Color& color,
	                               const unsigned int& size, const Vector2& offset)
	{
		std::string str = std::format("{}", value);
		renderText(window, camera, position, font, str, color, size, offset);
	}

	void RenderSFMLImpl::renderPolytope(sf::RenderWindow& window, Camera& camera, const std::vector<Vector2>& polytope,
	                                    const sf::Color& color,
	                                    const sf::Font& font, real pointSize, const unsigned int& indexSize,
	                                    bool showIndex)
	{
		Vector2 center = GeometryAlgorithm2D::calculateCenter(polytope);
		for (int i = 0; i < polytope.size(); ++i)
		{
			int j = (i + 1) % polytope.size();
			Vector2 offset = (polytope[i] - center).normal() * 0.3f;

			renderPoint(window, camera, polytope[i], color, pointSize);
			renderPoint(window, camera, polytope[j], color, pointSize);
			renderLine(window, camera, polytope[i], polytope[j], color);

			if (showIndex)
				renderInt(window, camera, polytope[i], font, i, color, indexSize, offset);
		}
	}

	void RenderSFMLImpl::renderPosition(sf::RenderWindow& window, Camera& camera, const Vector2& position,
	                                    const sf::Color& color, const sf::Font& font, const unsigned int& size,
	                                    const Vector2& screenOffset)
	{
		std::string str1 = std::format("({:.3f}, {:.3f})", position.x, position.y);
		renderText(window, camera, position, font, str1, color, size, screenOffset);
	}
}
