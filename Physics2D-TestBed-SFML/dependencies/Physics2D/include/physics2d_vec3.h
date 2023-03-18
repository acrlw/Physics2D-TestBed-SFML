#ifndef MATH_LINEAR_VECTOR3_H
#define MATH_LINEAR_VECTOR3_H

#include "physics2d_common.h"
namespace Physics2D
{
    struct Vec3
    {
        Vec3();
        Vec3(const real& x, const real& y, const real& z);
        Vec3(const real& fillNumber);
        Vec3(const Vec3& copy);
        Vec3(__m128 d);
        Vec3& operator=(const Vec3& copy);
        Vec3(Vec3&& other) = default;

        Vec3 operator+(const Vec3& rhs)const;
        Vec3 operator-(const Vec3& rhs)const;
        Vec3 operator-()const;
        Vec3 operator*(const real& factor)const;
        Vec3 operator/(const real& factor)const;

        Vec3& operator+=(const Vec3& rhs);
        Vec3& operator-=(const Vec3& rhs);
        Vec3& operator*=(const real& factor);
        Vec3& operator/=(const real& factor);
        real& operator[](uint8_t index);

        Vec3& fill(const real& number);
        Vec3& set(const real& x, const real& y, const real& z);
        Vec3& set(const Vec3& other);
        Vec3& set(__m128 d);
        Vec3& clear();
        Vec3& negate();
        Vec3& normalize();

        real magnitudeSquare()const;
        real magnitude()const;

        Vec3 normal()const;
        Vec3 negative()const;

        bool equal(const Vec3& rhs)const;
        bool fuzzyEqual(const Vec3& rhs, const real& epsilon = Constant::Eps)const;
        bool isOrigin(const real& epsilon = Constant::Eps)const;
        Vec3& swap(Vec3& other);

        real dot(const Vec3& rhs)const;
        Vec3 lerp(const Vec3& b, const real& t);
        Vec3& cross(const Vec3& rhs);

        static real dotProduct(const Vec3& lhs, const Vec3& rhs);
        static Vec3 crossProduct(const Vec3& lhs, const Vec3& rhs);

        static Vec3 lerp(const Vec3& a, const Vec3& b, const real& t);

        union
        {
            struct
            {
                float x;
                float y;
                float z;
                float w;
            };
            float f[4]{};
            __m128 data;
        };
    };
}
#endif
