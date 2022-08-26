#ifndef PHYSICS2D_SHAPE_CAPSULE_H
#define PHYSICS2D_SHAPE_CAPSULE_H
#include "../../geometry/shape.h"
namespace Physics2D
{
    class Capsule : public Shape
    {
    public:
        Capsule(real width = 0.0f, real height = 0.0f);
        bool contains(const Vec2& point, const real& epsilon) override;
        void scale(const real& factor) override;
        Vec2 center() const override;
        void set(real width, real height);
        void setWidth(real width);
        void setHeight(real height);
        real width()const;
        real height()const;
        Vec2 topLeft()const;
        Vec2 bottomLeft()const;
        Vec2 topRight()const;
        Vec2 bottomRight()const;
        Container::Vector<Vec2> boxVertices()const;
    private:
        real m_width;
        real m_height;
    };
}
#endif