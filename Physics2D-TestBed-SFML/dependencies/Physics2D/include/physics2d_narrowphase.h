#ifndef PHYSICS2D_NARROWPHASE_H
#define PHYSICS2D_NARROWPHASE_H
#include "physics2d_common.h"
#include "physics2d_shape.h"
#include "physics2d_simplex.h"

#include "physics2d_capsule.h"
#include "physics2d_circle.h"

#include "physics2d_edge.h"
#include "physics2d_ellipse.h"
#include "physics2d_polygon.h"
#include "physics2d_rectangle.h"


namespace Physics2D
{
	struct PHYSICS2D_API SimplexVertexWithOriginDistance
	{
		SimplexVertex vertex;
		real distance = 0.0f;
	};
	struct PHYSICS2D_API Feature
	{
		//circle and ellipse, use index 0
		//edge use index 0 and 1
		Vector2 vertex[2];
		Index index[2] = { INT_MAX, INT_MAX };
	};
	struct PHYSICS2D_API ClipVertex
	{
		Vector2 vertex;
		bool isClip = false;
		Vector2 clipperVertex;
		bool isFinalValid = false;
	};
	struct PHYSICS2D_API ContactPair
	{
		//contact pair1:
		//	points[0]: pointA
		//	points[1]: pointB

		//if there is second contact pair:
		//	points[2]: pointA
		//	points[3]: pointB
		std::array<Vector2, 4> points;
		uint32_t count = 0;
		void addContact(const Vector2& pointA, const Vector2& pointB)
		{
			assert(count <= 4);
			points[count++] = pointA;
			points[count++] = pointB;
		}
	};
	struct PHYSICS2D_API CollisionInfo
	{
		Vector2 normal;
		real penetration = 0;
		Simplex simplex;
		std::list<SimplexVertexWithOriginDistance> polytope;
	};
	class PHYSICS2D_API Narrowphase
	{
	public:
		static Simplex gjk(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const size_t& iteration = 12);
		static CollisionInfo epa(const Simplex& simplex, const ShapePrimitive& shapeA, const ShapePrimitive& shapeB,
			const size_t& iteration = 12, const real& epsilon = Constant::GeometryEpsilon);
		static SimplexVertex support(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const Vector2& direction);
		static std::pair<Vector2, Index> findFurthestPoint(const ShapePrimitive& shape, const Vector2& direction);
		static Vector2 findDirectionByEdge(const SimplexVertex& v1, const SimplexVertex& v2, bool pointToOrigin);
		static std::pair<Vector2, Index> findFurthestPoint(const Container::Vector<Vector2>& vertices, const Vector2& direction);
		static ContactPair generateContacts(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, CollisionInfo& info);

		static real gjkDistance(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, const size_t& iteration = 10);

		static void sat(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB);
		static void satPolygonVsPolygon(const Polygon& polygonA, const Transform& transformA, const Polygon& polygonB, const Transform& transformB);
		static void satPolygonVsCircle(const Polygon& polygonA, const Transform& transformA, const Circle& circleB, const Transform& transformB);
		static void satPolygonVsEllipse(const Polygon& polygonA, const Transform& transformA, const Ellipse& ellipseB, const Transform& transformB);
		static void satPolygonVsEdge(const Polygon& polygonA, const Transform& transformA, const Edge& edgeB, const Transform& transformB);
	private:
		static Feature findFeatures(const Simplex& simplex, const Vector2& normal, const ShapePrimitive& shape, const Index& AorB);

		static ContactPair clipIncidentEdge(std::array<ClipVertex, 2>& incEdge, std::array<Vector2, 2> refEdge, const Vector2& normal, bool swap);

		static ContactPair clipPolygonPolygon(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, 
			const Feature& featureA, const Feature& featureB, CollisionInfo& info);
		static ContactPair clipPolygonEdge(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, 
			const Feature& featureA, const Feature& featureB, CollisionInfo& info);
		static ContactPair clipPolygonCapsule(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, 
			const Feature& featureA, const Feature& featureB, CollisionInfo& info);
		static ContactPair clipPolygonRound(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, 
			const Feature& featureA, const Feature& featureB, CollisionInfo& info);

		static ContactPair clipEdgeCapsule(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, 
			const Feature& featureA, const Feature& featureB, CollisionInfo& info);
		static ContactPair clipEdgeRound(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, 
			const Feature& featureA, const Feature& featureB, CollisionInfo& info);

		static ContactPair clipCapsuleCapsule(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, 
			const Feature& featureA, const Feature& featureB, CollisionInfo& info);
		static ContactPair clipCapsuleRound(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, 
			const Feature& featureA, const Feature& featureB, CollisionInfo& info);

		static ContactPair clipRoundRound(const ShapePrimitive& shapeA, const ShapePrimitive& shapeB, 
			const Feature& featureA, const Feature& featureB, CollisionInfo& info);
		
	};

	
}
#endif