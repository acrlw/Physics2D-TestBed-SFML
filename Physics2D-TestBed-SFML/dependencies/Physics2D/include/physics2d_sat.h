#ifndef PHYSICS2D_SAT_H
#define PHYSICS2D_SAT_H

#include "physics2d_shape.h"
#include "physics2d_clip.h"

namespace Physics2D
{
    struct ProjectedPoint
    {
        Vec2 vertex;
        real value = 0;
        int index = -1;
        bool operator==(const ProjectedPoint& rhs);
    };
    struct ProjectedEdge
    {
        Vec2 vertex1;
        Vec2 vertex2;
    };
    struct ProjectedSegment
    {
        ProjectedPoint min;
        ProjectedPoint max;
        static std::tuple<ProjectedSegment, real> intersect(const ProjectedSegment& s1, const ProjectedSegment& s2);
    };
	
    struct SATResult
    {
        PointPair contactPair[2];
        uint32_t contactPairCount = 0;
        Vec2 normal;
        real penetration = 0;
        bool isColliding = false;
    };

    /// <summary>
    /// Separating Axis Theorem
    /// </summary>
    class SAT
    {
    public:
        static SATResult circleVsCapsule(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB);
        static SATResult circleVsSector(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB);
        static SATResult circleVsEdge(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB);
        static SATResult circleVsCircle(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB);
        static SATResult circleVsPolygon(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB);

        static SATResult polygonVsPolygon(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB);
		static SATResult polygonVsEdge(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB);
        static SATResult polygonVsCapsule(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB);
        static SATResult polygonVsSector(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB);
        
        static SATResult capsuleVsEdge(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB);
        static SATResult capsuleVsCapsule(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB);
		static SATResult capsuleVsSector(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB);

        static SATResult sectorVsSector(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB);

        static ProjectedSegment axisProjection(const Transform& transform, Shape* shape, Polygon* polygon, const Vec2& normal);
        static ProjectedSegment axisProjection(const Transform& transform, Shape* shape, Circle* circle, const Vec2& normal);
        static ProjectedSegment axisProjection(const Transform& transform, Shape* shape, Ellipse* ellipse, const Vec2& normal);
        static ProjectedSegment axisProjection(const Transform& transform, Shape* shape, Capsule* capsule, const Vec2& normal);
        static ProjectedSegment axisProjection(const Transform& transform, Shape* shape, Sector* sector, const Vec2& normal);

    };
    
}
#endif
