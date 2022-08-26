#ifndef PHYSICS2D_SHAPE_POLYGON_H
#define PHYSICS2D_SHAPE_POLYGON_H
#include "../../geometry/shape.h"
namespace Physics2D
{
    class Polygon : public Shape
    {

    public:
        Polygon();

        const Container::Vector<Vec2>& vertices() const;
        void append(const std::initializer_list<Vec2>& vertices);
        void append(const Vec2& vertex);
        Vec2 center()const override;
        void scale(const real& factor) override;
        bool contains(const Vec2& point, const real& epsilon = Constant::GeometryEpsilon) override;
    protected:
        Container::Vector<Vec2> m_vertices;
        void updateVertices();
    };
}
#endif