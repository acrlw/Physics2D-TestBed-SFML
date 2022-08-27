#ifndef PHYSICS2D_SHAPE_H
#define PHYSICS2D_SHAPE_H
#include "../../include/math/linear/linear.h"
#include "../../include/common/common.h"

namespace Physics2D
{
    class Shape
    {
        public:
            enum class Type
                {
                Point,
                Polygon,
                Circle,
                Ellipse,
            	Capsule,
                Edge,
                Curve,
                Sector
                };
            Type type()const
            {
                return m_type;
            }
            virtual void scale(const real& factor) = 0;
            virtual ~Shape() {};
            virtual bool contains(const Vec2& point, const real& epsilon = Constant::GeometryEpsilon) = 0;
            virtual Vec2 center()const = 0;
        protected:
            Type m_type;
    };

    /// <summary>
    /// Basic Shape Description in 2D Space.
    /// Including vertices/position/rotation of shape
    /// </summary>
    struct ShapeTransform
    {
		Shape* shape = nullptr;
        Transform transform;
    };
}
#endif
