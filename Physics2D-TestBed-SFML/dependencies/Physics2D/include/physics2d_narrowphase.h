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

namespace Physics2D::Narrowphase
{
	Simplex gjk(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const size_t& iteration = 20);
	void epa(const Simplex& simplex, const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
		const size_t& iteration = 20, const real& epsilon = Constant::GeometryEpsilon);
	SimplexVertex support(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const Vector2& direction);
	Vector2 findFurthestPoint(const ShapePrimitive& shape, const Vector2& direction);
	Vector2 calculateDirectionByEdge(const SimplexVertex& v1, const SimplexVertex& v2, bool pointToOrigin);
	std::pair<Vector2, Index> findFurthestPoint(const Container::Vector<Vector2>& vertices, const Vector2& direction);

	void sat(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
	void satPolygonVsPolygon(const Polygon& polygonA, const Polygon& polygonB);
	
}
#endif