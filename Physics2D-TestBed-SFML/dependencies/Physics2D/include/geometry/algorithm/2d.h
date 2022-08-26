#ifndef PHYSICS2D_ALGORITHM_GRAPHICS_2D
#define PHYSICS2D_ALGORITHM_GRAPHICS_2D

#include "../../math/linear/linear.h"
#include "../../common/common.h"
namespace Physics2D
{
	namespace GeometryAlgorithm2D
	{
		class Clipper
		{
		public:
			/// <summary>
			/// Sutherland Hodgman Polygon Clipping
			///	All points is stored in counter clock winding.
			///	By convention:
			///		p0 -> p1 -> p2 -> p0 constructs a triangle
			/// </summary>
			/// <param name="polygon"></param>
			/// <param name="clipRegion"></param>
			/// <returns></returns>
			static Container::Vector<Vec2> sutherlandHodgmentPolygonClipping(const Container::Vector<Vec2>& polygon, const Container::Vector<Vec2>& clipRegion);
		};
		/// <summary>
		/// Check if point a,b,c are collinear using triangle area method
		/// </summary>
		/// <param name="a">point a</param>
		/// <param name="b">point b</param>
		/// <param name="c">point c</param>
		/// <returns></returns>
		bool isCollinear(const Vec2& a, const Vec2& b, const Vec2& c);
		/// <summary>
		/// Check if point c is on line segment ab using line projection and set-union method
		/// </summary>
		/// <param name="a">end of segment a</param>
		/// <param name="b">end of segment b</param>
		/// <param name="c">point c</param>
		/// <returns></returns>
		bool isPointOnSegment(const Vec2& a, const Vec2& b, const Vec2& c);
		/// <summary>
		/// Check if point c is on line segment ab, given a,b,c is already collinear by calculating cross product
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <returns></returns>
		bool fuzzyIsPointOnSegment(const Vec2& a, const Vec2& b, const Vec2& c, const real& epsilon = Constant::GeometryEpsilon);
		bool fuzzyIsCollinear(const Vec2& a, const Vec2& b, const Vec2& c);
		/// <summary>
		/// Calculate intersected point between line ab and line cd.
		/// Return if there is a actual intersected point.
		/// Notices: overlapping is NOT considered as a kind of intersection situation in this function
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <param name="d"></param>
		/// <returns></returns>
		std::optional<Vec2> lineSegmentIntersection(const Vec2& a, const Vec2& b, const Vec2& c, const Vec2& d);
		/// <summary>
		/// line intersection
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <param name="d"></param>
		/// <returns></returns>
		Vec2 lineIntersection(const Vec2& p1, const Vec2& p2, const Vec2& q1, const Vec2& q2);
		/// <summary>
		/// Calculate the center of circum-circle from triangle abc
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <returns></returns>
		std::optional<Vec2> triangleCircumcenter(const Vec2& a, const Vec2& b, const Vec2& c);
		/// <summary>
		/// Calculate the center of inscribed-circle from triangle abc
		/// If a,b,c can not form a triangle, return nothing
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <returns></returns>
		std::optional<Vec2> triangleIncenter(const Vec2& a, const Vec2& b, const Vec2& c);
		/// <summary>
		/// Calculate circum-circle given three points that can form a triangle
		/// If a,b,c can not form a triangle, return nothing
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <returns></returns>
		std::optional<std::tuple<Vec2, real>> calculateCircumcircle(const Vec2& a, const Vec2& b, const Vec2& c);
		/// <summary>
		/// Calculate inscribed circle given three points that can form a triangle
		/// If a,b,c can not form a triangle, return nothing
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="c"></param>
		/// <returns></returns>
		std::optional<std::tuple<Vec2, real>> calculateInscribedCircle(const Vec2& a, const Vec2& b, const Vec2& c);
		/// <summary>
		/// Check if a polygon is convex
		/// </summary>
		/// <param name="vertices"></param>
		/// <returns></returns>
		bool isConvexPolygon(const Container::Vector<Vec2>& vertices);
		/// <summary>
		/// Convex hull algorithm: Graham Scan. Given a series of points, find the convex polygon that can contains all of these points.
		/// </summary>
		/// <param name="vertices"></param>
		/// <returns></returns>
		Container::Vector<Vec2> grahamScan(const Container::Vector<Vec2>& vertices);
		/// <summary>
		/// calculate point on line segment ab that is the shortest length to point p
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <returns></returns>
		Vec2 pointToLineSegment(const Vec2& a, const Vec2& b, const Vec2& p);
		/// <summary>
		/// Calculate point on ellipse that is the shortest length to point p(aka projection point)
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="p"></param>
		/// <returns></returns>
		Vec2 shortestLengthPointOfEllipse(const real& a, const real& b, const Vec2& p, const real& epsilon = 0.00000001f);
		/// <summary>
		/// Calculate the centroid of triangle.
		/// </summary>
		/// <param name="a1"></param>
		/// <param name="a2"></param>
		/// <param name="a3"></param>
		/// <returns></returns>
		Vec2 triangleCentroid(const Vec2& a1, const Vec2& a2, const Vec2& a3);
		/// <summary>
		/// Calculate the area of triangle use cross product
		/// </summary>
		/// <param name="a1"></param>
		/// <param name="a2"></param>
		/// <param name="a3"></param>
		/// <returns></returns>
		real triangleArea(const Vec2& a1, const Vec2& a2, const Vec2& a3);
		/// <summary>
		/// Calculate mass center of 'convex' polygon
		/// </summary>
		/// <param name="vertices"></param>
		/// <returns></returns>
		Vec2 calculateCenter(const Container::Vector<Vec2>& vertices);
		/// <summary>
		/// Calculate two points on line segment and ellipse respectively. The length of two points is the shortest distance of line segment and ellipse
		/// </summary>
		/// <param name="a">major axis a</param>
		/// <param name="b">minor axis b</param>
		/// <param name="p1">line segment point 1</param>
		/// <param name="p2">line segment point 2</param>
		/// <returns></returns>
		std::tuple<Vec2, Vec2> shortestLengthLineSegmentEllipse(const real& a, const real& b, const Vec2& p1, const Vec2& p2);
		/// <summary>
		/// Calculate point on line segment ab, if point 'p' can cast ray in 'dir' direction on line segment ab.
		/// Algorithm from wikipedia 'Line-line intersection'
		/// </summary>
		/// <param name="p">ray start point</param>
		/// <param name="dir">ray direction</param>
		/// <param name="a">line segment point a</param>
		/// <param name="b">line segment point b</param>
		/// <returns></returns>
		std::optional<Vec2> raycast(const Vec2& p, const Vec2& dir, const Vec2& a, const Vec2& b);
		std::optional<std::pair<Vec2, Vec2>> raycastAABB(const Vec2& p, const Vec2& dir, const Vec2& topLeft, const Vec2& bottomRight);
		bool isPointOnAABB(const Vec2& p, const Vec2& topLeft, const Vec2& bottomRight);
		/// <summary>
		/// Rotate point 'p' around point 'center' by 'angle' degrees
		/// </summary>
		/// <param name="p">source point</param>
		/// <param name="center">center point</param>
		/// <param name="angle">rotate angle</param>
		/// <returns></returns>
		Vec2 rotate(const Vec2& p, const Vec2& center, const real& angle);
		/// <summary>
		/// Calculate the projection axis of ellipse in user-define direction.
		/// Return the maximum point in ellipse
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <param name="direction"></param>
		/// <returns></returns>
		Vec2 calculateEllipseProjectionPoint(const real& a, const real& b, const Vec2& direction);
		Vec2 calculateCapsuleProjectionPoint(const real& width, const real& height, const Vec2& direction);
		Vec2 calculateSectorProjectionPoint(const real& startRadian, const real& spanRadian, const real& radius, const Vec2& direction);
		bool triangleContainsOrigin(const Vec2& a, const Vec2& b, const Vec2& c);
		bool isPointOnSameSide(const Vec2& edgePoint1, const Vec2& edgePoint2, const Vec2& refPoint, const Vec2 targetPoint);
	};
}
#endif
