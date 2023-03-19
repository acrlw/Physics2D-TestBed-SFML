#include "physics2d_narrowphase.h"

namespace Physics2D::Narrowphase
{
	Simplex gjk(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const size_t& iteration)
	{
		Simplex simplex;
		Vector2 direction = shapeB.transform.position - shapeA.transform.position;

		if (direction.fuzzyEqual({ 0, 0 }))
			direction.set(1, 1);
		//first
		SimplexVertex vertex = support(shapeA, shapeB, direction);
		simplex.addSimplexVertex(vertex);
		//second
		direction.negate();
		vertex = support(shapeA, shapeB, direction);
		simplex.addSimplexVertex(vertex);
		//check 1d simplex(line segment) contains origin
		if (simplex.containsOrigin())
			return simplex;
		//third
		size_t iter = 0;
		while (iter <= iteration)
		{
			//default closest edge is index 0 and index 1
			direction = calculateDirectionByEdge(simplex.vertices[0], simplex.vertices[1], true);
			vertex = support(shapeA, shapeB, direction);

			//find repeated vertex
			if (simplex.contains(vertex))
				break;

			//vertex does not pass origin
			if (vertex.result.dot(direction) <= 0)
				break;

			simplex.addSimplexVertex(vertex);

			//use barycentric coordinates to check contains origin and find closest edge
			SimplexVertex va = simplex.vertices[0];
			SimplexVertex vb = simplex.vertices[1];
			SimplexVertex vc = simplex.vertices[2];
			Vector2 a = va.result;
			Vector2 b = vb.result;
			Vector2 c = vc.result;
			Vector2 ab = b - a;
			Vector2 ac = c - a;
			Vector2 bc = c - b;
			real ab_length = ab.length();
			real ac_length = ab.length();
			real bc_length = ab.length();

			real u_ab = -a.dot(ab.normal()) / ab_length;
			real u_ac = -a.dot(ac.normal()) / ac_length;
			real u_bc = -b.dot(bc.normal()) / bc_length;
			real v_ab = 1 - u_ab;
			real v_ac = 1 - u_ac;
			real v_bc = 1 - u_bc;
			/*
			 * [Ax Bx Cx][u] = [0]
			 * [Ay By Cy][v] = [0]
			 * [ 1  1  1][w] = [1]
			 * solve for u,v,w
			 */
			real u, v, w;
			real det = a.y * b.x - a.x * b.y + a.x * c.y - a.y * c.x + b.y * c.x - c.y * b.x;
			assert(det != 0.0f);
			u = (b.y * c.x - c.y * b.x) / det;
			v = (c.y * a.x - a.y * c.x) / det;
			w = 1 - u - v;

			if (u_ab > 0 && v_ab > 0 && w <= 0)
			{
				//in region AB, remove c
				simplex.vertices[0] = va;
				simplex.vertices[1] = vb;
			}
			else if (u_ac > 0 && v_ac > 0 && v <= 0)
			{
				//in region AC, remove b
				simplex.vertices[0] = va;
				simplex.vertices[1] = vc;
			}
			else if (u_bc > 0 && v_bc > 0 && u <= 0)
			{
				//in region BC, remove a
				simplex.vertices[0] = vb;
				simplex.vertices[1] = vc;
			}
			else if (u > 0 && v > 0 && w > 0)
			{
				//in region ABC, origin is inside simplex
				simplex.isContainOrigin = true;

				//reorder simplex index, so that the closest edge is index 0 and index 1
				//h means height
				real h_u = u / bc_length;
				real h_v = v / ac_length;
				real h_w = w / ab_length;
				real min = Math::tripleMin(h_u, h_v, h_w);
				if (min == h_u)
				{
					simplex.vertices[0] = vc;
					simplex.vertices[2] = va;
				}
				else if (min == h_v)
				{
					simplex.vertices[1] = vc;
					simplex.vertices[2] = vb;
				}
				//else, min == h_w, default is abc, do nothing

				return simplex;
			}

			simplex.removeByIndex(2);

			iter++;
		}
		return simplex;
	}

	void epa(const Simplex& simplex, const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const size_t& iteration, const real& epsilon)
	{
		std::list<SimplexVertex> polytope;

	}

	SimplexVertex support(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const Vector2& direction)
	{
		return SimplexVertex(findFurthestPoint(shapeA, direction), findFurthestPoint(shapeB, direction * -1));
	}

	Vector2 findFurthestPoint(const ShapePrimitive& shape, const Vector2& direction)
	{
		Vector2 target;
		Matrix2x2 rot(-shape.transform.rotation);
		Vector2 rot_dir = rot.multiply(direction);
		switch (shape.shape->type())
		{
		case Shape::Type::Polygon:
		{
			const Polygon* polygon = static_cast<const Polygon*>(shape.shape);
			auto [vertex, index] = findFurthestPoint(polygon->vertices(), rot_dir);
			target = vertex;
			break;
		}
		case Shape::Type::Circle:
		{
			const Circle* circle = static_cast<const Circle*>(shape.shape);
			return direction.normal() * circle->radius() + shape.transform.position;
		}
		case Shape::Type::Ellipse:
		{
			const Ellipse* ellipse = static_cast<const Ellipse*>(shape.shape);
			target = GeometryAlgorithm2D::calculateEllipseProjectionPoint(ellipse->A(), ellipse->B(), rot_dir);
			break;
		}
		case Shape::Type::Edge:
		{
			const Edge* edge = static_cast<const Edge*>(shape.shape);
			real dot1 = Vector2::dotProduct(edge->startPoint(), direction);
			real dot2 = Vector2::dotProduct(edge->endPoint(), direction);
			target = dot1 > dot2 ? edge->startPoint() : edge->endPoint();
			break;
		}
		case Shape::Type::Point:
		{
			return static_cast<const Point*>(shape.shape)->position();
		}
		case Shape::Type::Capsule:
		{
			const Capsule* capsule = static_cast<const Capsule*>(shape.shape);
			target = GeometryAlgorithm2D::calculateCapsuleProjectionPoint(capsule->width(), capsule->height(), rot_dir);
			break;
		}
		case Shape::Type::Sector:
		{
			const Sector* sector = static_cast<const Sector*>(shape.shape);
			target = GeometryAlgorithm2D::calculateSectorProjectionPoint(sector->startRadian(), sector->spanRadian(), sector->radius(), rot_dir);
			break;
		}
		case Shape::Type::Curve:
			assert(false && "Mapping for curve is not supported.");
			break;
		}
		rot.set(shape.transform.rotation);
		target = rot.multiply(target);
		target += shape.transform.position;
		return target;
	}

	Vector2 calculateDirectionByEdge(const SimplexVertex& v1, const SimplexVertex& v2, bool pointToOrigin)

	{
		const Vector2 p1 = v1.result;
		const Vector2 p2 = v2.result;
		const Vector2 ao = p1 * -1;
		const Vector2 ab = p2 - p1;
		Vector2 perpendicularOfAB = ab.perpendicular();
		if ((Vector2::dotProduct(ao, perpendicularOfAB) < 0 && pointToOrigin) || (
			Vector2::dotProduct(ao, perpendicularOfAB) > 0 && !pointToOrigin))
			perpendicularOfAB.negate();
		return perpendicularOfAB;
	}
	std::pair<Vector2, Index> findFurthestPoint(const Container::Vector<Vector2>& vertices, const Vector2& direction)
	{
		real max = Constant::NegativeMin;
		Vector2 target;
		size_t index = 0;
		for (size_t i = 0; i < vertices.size(); i++)
		{
			real result = Vector2::dotProduct(vertices[i], direction);
			if (max < result)
			{
				max = result;
				target = vertices[i];
				index = i;
			}
		}
		return std::make_pair(target, index);
	}

	void sat(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB)
	{

	}
}
