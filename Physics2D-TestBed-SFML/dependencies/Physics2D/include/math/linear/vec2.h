#ifndef MATH_LINEAR_Vec2_H
#define MATH_LINEAR_Vec2_H
#include "../../common/common.h"
namespace Physics2D
{
    struct Vec2
    {
        Vec2();
        Vec2(const real& fillNumber);
        Vec2(const real& x, const real& y);
        Vec2(const Vec2& copy);
        Vec2& operator=(const Vec2& copy);
        Vec2(Vec2&& other) = default;

        real& operator[](uint8_t index);

        Vec2 operator+(const Vec2& rhs)const;
        Vec2 operator-(const Vec2& rhs)const;
        Vec2 operator-()const;
        Vec2 operator*(const real& factor)const;
        Vec2 operator/(const real& factor)const;

        Vec2& operator+=(const Vec2& rhs);
        Vec2& operator-=(const Vec2& rhs);
        Vec2& operator*=(const real& factor);
        Vec2& operator/=(const real& factor);

        bool operator==(const Vec2& rhs)const;
        bool operator!=(const Vec2& rhs)const;
        bool equal(const Vec2& rhs)const;
        bool fuzzyEqual(const Vec2& rhs, const real& epsilon = Constant::Eps)const;
        bool isOrigin(const real& epsilon = Constant::Eps)const;

        real magnitudeSquare()const;
        real magnitude()const;
        real theta()const;
        Vec2 normal()const;
        Vec2 negative()const;

        Vec2& fill(const real& fillNumber);
        Vec2& set(const real& x, const real& y);
        Vec2& set(const Vec2& copy);
        Vec2& clear();
        Vec2& negate();
        Vec2& swap(Vec2& other) noexcept;

        Vec2& normalize();
        Vec2 perpendicular()const;


        real dot(const Vec2& rhs)const;
        real cross(const Vec2& rhs)const;

        static real dotProduct(const Vec2& lhs, const Vec2& rhs);
        static real crossProduct(const Vec2& lhs, const Vec2& rhs);
        static real crossProduct(const real& x1, const real& y1, const real& x2, const real& y2);
        static Vec2 crossProduct(const real& lhs, const Vec2& rhs);
        static Vec2 crossProduct(const Vec2& lhs, const real& rhs);
        static Vec2 lerp(const Vec2& a, const Vec2& b, const real& t);

        union
        {
            struct
            {
                float x;
                float y;
            };
            float f[2]{};
        };
    };
}
#endif
