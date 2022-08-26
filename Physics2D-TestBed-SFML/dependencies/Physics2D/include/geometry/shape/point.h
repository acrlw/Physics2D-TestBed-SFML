#ifndef PHYSICS2D_SHAPE_POINT_H
#define PHYSICS2D_SHAPE_POINT_H
#include "../../geometry/shape.h"
namespace Physics2D
{
    class Point : public Shape
    {
    public:
        Point();

        Vec2 position() const;
        void setPosition(const Vec2& pos);
        Vec2 center()const override;
        void scale(const real& factor) override;
        bool contains(const Vec2& point, const real& epsilon = Constant::GeometryEpsilon) override;
    private:
        Vec2 m_position;
    };
}
#endif