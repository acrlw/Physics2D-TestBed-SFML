#ifndef PHYSICS2D_SHAPE_CURVE_H
#define PHYSICS2D_SHAPE_CURVE_H
#include "physics2d_shape.h"
namespace Physics2D
{
    class Curve : public Shape
    {

    public:
        Curve();
        void set(const Vec2& start, const Vec2& control1, const Vec2& control2, const Vec2& end);

        Vec2 startPoint() const;
        void setStartPoint(const Vec2& startPoint);

        Vec2 control1() const;
        void setControl1(const Vec2& control1);

        Vec2 control2() const;
        void setControl2(const Vec2& control2);

        Vec2 endPoint() const;
        void setEndPoint(const Vec2& endPoint);

        void scale(const real& factor) override;
        bool contains(const Vec2& point, const real& epsilon = Constant::GeometryEpsilon) override;
        Vec2 center()const override;
    private:
        Vec2 m_startPoint;
        Vec2 m_control1;
        Vec2 m_control2;
        Vec2 m_endPoint;
    };
}
#endif