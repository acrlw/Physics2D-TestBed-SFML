#ifndef MATH_LINEAR_VECTOR4_H
#define MATH_LINEAR_VECTOR4_H
#include "../../math/linear/vec3.h"
namespace Physics2D
{
    struct Vec4
    {
        Vec4();
        Vec4(__m128 d);
        Vec4(const real& fillNumber);
        Vec4(const real& x, const real& y, const real& z, const real& w);
        Vec4& operator=(const Vec4& copy);
        Vec4& operator=(const Vec3& xyz);
        Vec4(const Vec4& copy) = default;
        Vec4(Vec4&& other) = default;
        Vec4(const Vec3& xyz);

        Vec4 operator+(const Vec4& rhs)const;
        Vec4 operator-(const Vec4& rhs)const;
        Vec4 operator-()const;
        Vec4 operator*(const real& factor)const;
        Vec4 operator/(const real& factor)const;

        Vec4& operator+=(const Vec4& rhs);
        Vec4& operator-=(const Vec4& rhs);
        Vec4& operator*=(const real& factor);
        Vec4& operator/=(const real& factor);

        Vec4& fill(const real& number);
        Vec4& set(const real& x, const real& y, const real& z, const real& w);
        Vec4& set(const Vec4& other);
        Vec4& set(__m128 d);

        Vec4& set(const Vec3& xyz);
        Vec4& set(const Vec3& xyz, const real& w);
        Vec4& clear();
        Vec4& negate();
        Vec4& normalize();

        real magnitudeSquare()const;
        real magnitude()const;

        Vec4 normal()const;
        Vec4 negative()const;

        Vec3 xyz()const;

        bool equal(const Vec4& rhs)const;
        bool fuzzyEqual(const Vec4& rhs, const real& epsilon = Constant::Eps)const;
        bool isOrigin(const real& epsilon = Constant::Eps)const;
        Vec4& swap(Vec4& other);

        real dot(const Vec4& rhs)const;


        static real dotProduct(const Vec4& lhs, const Vec4& rhs);
        static Vec4 lerp(const Vec4& a, const Vec4& b, const real& t);

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