#ifndef PHYSICS2D_SHAPE_CIRCLE_H
#define PHYSICS2D_SHAPE_CIRCLE_H
#include "physics2d_shape.h"
namespace Physics2D
{
    class PHYSICS2D_API Circle : public Shape
    {

    public:
        Circle(real radius = 0);

        real radius() const;
        void setRadius(const real& radius);
        void scale(const real& factor) override;
        bool contains(const Vector2& point, const real& epsilon = Constant::GeometryEpsilon) override;
        Vector2 center()const override;
    private:
        real m_radius;
    };
}
#endif