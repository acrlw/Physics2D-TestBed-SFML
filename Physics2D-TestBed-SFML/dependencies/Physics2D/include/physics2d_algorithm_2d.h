#ifndef PHYSICS2D_ALGORITHM_GRAPHICS_2D
#define PHYSICS2D_ALGORITHM_GRAPHICS_2D

#include "physics2d_linear.h"
#include "physics2d_common.h"
namespace Physics2D
{
	namespace GeometryAlgorithm2D
	{
		class Clipper
		{
		public:
			/**
			 * \brief Sutherland Hodgman Polygon Clipping. All points is stored in counter clock winding.\n
			 * By convention:\n
			 *		p0 -> p1 -> p2 -> p0 constructs a triangle
			 * \param polygon 
			 * \param clipRegion 
			 * \return 
			 */
			static Container::Vector<Vector2> sutherlandHodgmentPolygonClipping(const Container::Vector<Vector2>& polygon, const Container::Vector<Vector2>& clipRegion);
		};

		/**
		 * \brief Check if point a,b,c are collinear using triangle area method
		 * \param a point a
		 * \param b point b
		 * \param c point c
		 * \return 
		 */
		bool isCollinear(const Vector2& a, const Vector2& b, const Vector2& c);
		/**
		 * \brief Check if point c is on line segment ab using line projection and set-union method
		 * \param a end of segment a
		 * \param b end of segment b
		 * \param c point c
		 * \return 
		 */
		bool isPointOnSegment(const Vector2& a, const Vector2& b, const Vector2& c);
		/**
		 * \brief Check if point c is on line segment ab, given a,b,c is already collinear by calculating cross product
		 * \param a 
		 * \param b 
		 * \param c 
		 * \param epsilon 
		 * \return 
		 */
		bool fuzzyIsPointOnSegment(const Vector2& a, const Vector2& b, const Vector2& c, const real& epsilon = Constant::GeometryEpsilon);
		bool fuzzyIsCollinear(const Vector2& a, const Vector2& b, const Vector2& c);
		/**
		 * \brief Calculate intersected point between line ab and line cd.\n
		 * Notices: overlapping is NOT considered as a kind of intersection situation in this function
		 * \param a 
		 * \param b 
		 * \param c 
		 * \param d 
		 * \return if there is a actual intersected point.
		 */
		std::optional<Vector2> lineSegmentIntersection(const Vector2& a, const Vector2& b, const Vector2& c, const Vector2& d);
		/**
		 * \brief line intersection
		 * \param p1 
		 * \param p2 
		 * \param q1 
		 * \param q2 
		 * \return 
		 */
		Vector2 lineIntersection(const Vector2& p1, const Vector2& p2, const Vector2& q1, const Vector2& q2);
		/**
		 * \brief Calculate the center of circum-circle from triangle abc
		 * \param a 
		 * \param b 
		 * \param c 
		 * \return 
		 */
		std::optional<Vector2> triangleCircumcenter(const Vector2& a, const Vector2& b, const Vector2& c);
		/**
		 * \brief Calculate the center of inscribed-circle from triangle abc. If a,b,c can not form a triangle, return nothing
		 * \param a 
		 * \param b 
		 * \param c 
		 * \return 
		 */
		std::optional<Vector2> triangleIncenter(const Vector2& a, const Vector2& b, const Vector2& c);
		/**
		 * \brief Calculate circum-circle given three points that can form a triangle. If a,b,c can not form a triangle, return nothing
		 * \param a 
		 * \param b 
		 * \param c 
		 * \return 
		 */
		std::optional<std::tuple<Vector2, real>> calculateCircumcircle(const Vector2& a, const Vector2& b, const Vector2& c);
		/**
		 * \brief Calculate inscribed circle given three points that can form a triangle. If a,b,c can not form a triangle, return nothing.
		 * \param a 
		 * \param b 
		 * \param c 
		 * \return 
		 */
		std::optional<std::tuple<Vector2, real>> calculateInscribedCircle(const Vector2& a, const Vector2& b, const Vector2& c);
		/**
		 * \brief Check if a polygon is convex
		 * \param vertices 
		 * \return 
		 */
		bool isConvexPolygon(const Container::Vector<Vector2>& vertices);
		/**
		 * \brief Convex hull algorithm: Graham Scan. Given a series of points, find the convex polygon that can contains all of these points.
		 * \param vertices 
		 * \return 
		 */
		Container::Vector<Vector2> grahamScan(const Container::Vector<Vector2>& vertices);
		/**
		 * \brief Calculate point on ellipse that is the shortest length to point p(aka projection point).
		 * \param a 
		 * \param b 
		 * \param p 
		 * \param epsilon 
		 * \return 
		 */
		Vector2 shortestLengthPointOfEllipse(const real& a, const real& b, const Vector2& p, const real& epsilon = Constant::GeometryEpsilon);
		/**
		 * \brief Calculate the centroid of triangle.
		 * \param a1 
		 * \param a2 
		 * \param a3 
		 * \return 
		 */
		Vector2 triangleCentroid(const Vector2& a1, const Vector2& a2, const Vector2& a3);
		/**
		 * \brief Calculate the area of triangle use cross product.
		 * \param a1 
		 * \param a2 
		 * \param a3 
		 * \return 
		 */
		real triangleArea(const Vector2& a1, const Vector2& a2, const Vector2& a3);
		/**
		 * \brief Calculate mass center of 'convex' polygon
		 * \param vertices 
		 * \return 
		 */
		Vector2 calculateCenter(const Container::Vector<Vector2>& vertices);
		/**
		 * \brief Calculate two points on line segment and ellipse respectively. The length of two points is the shortest distance of line segment and ellipse
		 * \param a major axis a
		 * \param b minor axis b
		 * \param p1 line segment point 1
		 * \param p2 line segment point 2
		 * \return 
		 */
		std::tuple<Vector2, Vector2> shortestLengthLineSegmentEllipse(const real& a, const real& b, const Vector2& p1, const Vector2& p2);
		/**
		 * \brief Calculate point on line segment ab, if point 'p' can cast ray in 'dir' direction on line segment ab. \n
		 * Algorithm from wikipedia Line-line intersection.
		 * \param p ray start point
		 * \param dir ray direction
		 * \param a line segment point a
		 * \param b line segment point b
		 * \return 
		 */
		std::optional<Vector2> raycast(const Vector2& p, const Vector2& dir, const Vector2& a, const Vector2& b);
		std::optional<std::pair<Vector2, Vector2>> raycastAABB(const Vector2& p, const Vector2& dir, const Vector2& topLeft, const Vector2& bottomRight);
		bool isPointOnAABB(const Vector2& p, const Vector2& topLeft, const Vector2& bottomRight);
		/**
		 * \brief Rotate point 'p' around point 'center' by 'angle' degrees
		 * \param p 
		 * \param center 
		 * \param angle 
		 * \return 
		 */
		Vector2 rotate(const Vector2& p, const Vector2& center, const real& angle);
		/**
		 * \brief Calculate the projection axis of ellipse in user-define direction.
		 * \param a 
		 * \param b 
		 * \param direction 
		 * \return the maximum point in ellipse
		 */
		Vector2 calculateEllipseProjectionPoint(const real& a, const real& b, const Vector2& direction);
		Vector2 calculateCapsuleProjectionPoint(const real& width, const real& height, const Vector2& direction);
		Vector2 calculateSectorProjectionPoint(const real& startRadian, const real& spanRadian, const real& radius, const Vector2& direction);
		bool triangleContainsOrigin(const Vector2& a, const Vector2& b, const Vector2& c);
		bool isPointOnSameSide(const Vector2& edgePoint1, const Vector2& edgePoint2, const Vector2& refPoint, const Vector2 targetPoint);
		/**
		 * \brief calculate point on line segment ab that is the shortest length to point p
		 * \param a point a
		 * \param b point b
		 * \param p target point
		 * \return 
		 */
		Vector2 pointToLineSegment(const Vector2& a, const Vector2& b, const Vector2& p);
	};
}
#endif
