#ifndef PHYSICS2D_SHAPE_EDGE_H
#define PHYSICS2D_SHAPE_EDGE_H
#include "physics2d_shape.h"
namespace Physics2D
{
    class Edge : public Shape
    {

    public:
        Edge();

        void set(const Vec2& start, const Vec2& end);

        Vec2 startPoint()const;
        void setStartPoint(const Vec2& start);

        Vec2 endPoint()const;
        void setEndPoint(const Vec2& end);
        void scale(const real& factor) override;
        bool contains(const Vec2& point, const real& epsilon = Constant::GeometryEpsilon) override;
        Vec2 center()const override;
        Vec2 normal()const;
        void setNormal(const Vec2& normal);
    private:
        Vec2 m_startPoint;
        Vec2 m_endPoint;
        Vec2 m_normal;
    };
}
#endif