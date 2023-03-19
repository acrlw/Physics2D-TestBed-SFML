#ifndef PHYSICS2D_NARROWPHASE_H
#define PHYSICS2D_NARROWPHASE_H
#include "physics2d_common.h"
#include "physics2d_shape.h"
#include "physics2d_simplex.h"

#include "physics2d_capsule.h"
#include "physics2d_circle.h"
#include "physics2d_curve.h"
#include "physics2d_edge.h"
#include "physics2d_ellipse.h"
#include "physics2d_point.h"
#include "physics2d_polygon.h"
#include "physics2d_rectangle.h"
#include "physics2d_sector.h"

namespace Physics2D
{
	struct SimplexVertexWithOriginDistance
	{
		SimplexVertex vertex;
		real distance = 0.0f;
	};
	struct ContactPair
	{
		Vector2 points[4];
		uint32_t count = 0;
		void addContact(const Vector2& pointA, const Vector2& pointB)
		{
			assert(count <= 4);
			points[count++] = pointA;
			points[count++] = pointB;
		}
	};
	struct CollisionInfo
	{
		Vector2 normal;
		real penetration = 0;
		Simplex simplex;
	};
	class Narrowphase
	{
	public:
		static Simplex gjk(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const size_t& iteration = 8);
		static CollisionInfo epa(const Simplex& simplex, const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
			const size_t& iteration = 8, const real& epsilon = Constant::GeometryEpsilon);
		static SimplexVertex support(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const Vector2& direction);
		static std::pair<Vector2, Index> findFurthestPoint(const ShapePrimitive& shape, const Vector2& direction);
		static Vector2 calculateDirectionByEdge(const SimplexVertex& v1, const SimplexVertex& v2, bool pointToOrigin);
		static std::pair<Vector2, Index> findFurthestPoint(const Container::Vector<Vector2>& vertices, const Vector2& direction);
		static ContactPair clip(const Simplex& simplex, const Vector2& normal, const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);

		static real gjkDistance(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const size_t& iteration = 8);

		static void sat(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
		static void satPolygonVsPolygon(const Polygon& polygonA, const Transform& transformA, const Polygon& polygonB, const Transform& transformB);
		static void satPolygonVsCircle(const Polygon& polygonA, const Transform& transformA, const Circle& circleB, const Transform& transformB);
		static void satPolygonVsEllipse(const Polygon& polygonA, const Transform& transformA, const Ellipse& ellipseB, const Transform& transformB);
		static void satPolygonVsEdge(const Polygon& polygonA, const Transform& transformA, const Edge& edgeB, const Transform& transformB);
	private:
		static std::tuple<bool, Vector2, Index, Index> dumpFeatures(const Simplex& simplex, const Vector2& normal, const ShapePrimitive& shape, const Index& AorB);
	};

}
#endif