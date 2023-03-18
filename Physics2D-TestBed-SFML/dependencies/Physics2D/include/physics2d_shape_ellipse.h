#ifndef PHYSICS2D_SHAPE_ELLIPSE_H
#define PHYSICS2D_SHAPE_ELLIPSE_H
#include "physics2d_shape.h"
namespace Physics2D
{
    class Ellipse : public Shape
    {

    public:
        Ellipse(const real& width = 0, const real& height = 0);
        void set(const Vec2& leftTop, const Vec2& rightBottom);
        void set(const real& width, const real& height);

        real width()const;
        void setWidth(const real& width);

        real height()const;
        void setHeight(const real& height);

        void scale(const real& factor) override;
        bool contains(const Vec2& point, const real& epsilon = Constant::GeometryEpsilon) override;
        Vec2 center()const override;
        real A()const;
        real B()const;
        real C()const;
    private:
        real m_width;
        real m_height;
    };
}
#endif