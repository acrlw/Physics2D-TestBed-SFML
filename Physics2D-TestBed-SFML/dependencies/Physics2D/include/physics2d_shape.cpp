#include "physics2d_shape.h"

namespace Physics2D
{
    bool ShapePrimitive::contains(const Vector2& point, const real& epsilon)
    {
        return false;
    }

    bool ShapePrimitive::contains(Shape* shape, const Vector2& transform, const real& rotation, const Vector2& point, const real& epsilon)
    {
        return false;
    }
}

