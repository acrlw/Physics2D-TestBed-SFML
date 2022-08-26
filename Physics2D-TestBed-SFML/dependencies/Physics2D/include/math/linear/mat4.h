#ifndef MATH_LINEAR_MATRIX4X4_H
#define MATH_LINEAR_MATRIX4X4_H
#include "../../common/common.h"
#include "../../math/math.h"
#include "../../math/linear/mat3.h"
#include "../../math/linear/vec4.h"
namespace Physics2D
{
    struct Mat4
    {
        Mat4();
        Mat4(const Vec4& r1, const Vec4& r2, const Vec4& r3, const Vec4& r4);

        Mat4(
            const real& e00, const real& e01, const real& e02, const real& e03,
            const real& e10, const real& e11, const real& e12, const real& e13,
            const real& e20, const real& e21, const real& e22, const real& e23,
            const real& e30, const real& e31, const real& e32, const real& e33);

        Mat4(const Mat3& mat);
        Mat4(const Mat4& mat) = default;

        Mat4(Mat4&& other) = default;

        Mat4& operator=(const Mat3& rhs);
        Mat4& operator=(const Mat4& rhs);

        Mat4 operator+(const Mat4& rhs)const;
        Mat4 operator-(const Mat4& rhs)const;
        Vec4 operator[](uint8_t index)const;
        Mat4& operator+=(const Mat4& rhs);
        Mat4& operator-=(const Mat4& rhs);
        Mat4& operator*=(const real& factor);
        Mat4& operator/=(const real& factor);

        Vec4 c0()const;
        Vec4 c1()const;
        Vec4 c2()const;
        Vec4 c3()const;

        real e00()const;
        real e01()const;
        real e02()const;
        real e03()const;

        real e10()const;
        real e11()const;
        real e12()const;
        real e13()const;

        real e20()const;
        real e21()const;
        real e22()const;
        real e23()const;

        real e30()const;
        real e31()const;
        real e32()const;
        real e33()const;

        Mat4& set(
            const real& e00, const real& e01, const real& e02, const real& e03,
            const real& e10, const real& e11, const real& e12, const real& e13,
            const real& e20, const real& e21, const real& e22, const real& e23,
            const real& e30, const real& e31, const real& e32, const real& e33);

        Mat4& set(const Vec4& c1, const Vec4& c2, const Vec4& c3, const Vec4& c4);
        Mat4& set(const Mat4& other);

        Mat4& set(const Mat3& other);

        Mat4& clear();

        Vec4 multiply(const Vec4& rhs)const;
        Mat4& multiply(const Mat4& rhs);
        real determinant()const;
        Mat4& transpose();
        Mat4& invert();

        static Mat4 identity();
        static Mat4 multiply(const Mat4& lhs, const Mat4& rhs);
        static Vec4 multiply(const Mat4& lhs, const Vec4& rhs);
        static Vec4 multiply(const Vec4& lhs, const Mat4& rhs);
        static real determinant(const Mat4& mat);
        static bool invert(Mat4& mat);

        union
        {
            struct
            {
                Vec4 r0;
                Vec4 r1;
                Vec4 r2;
                Vec4 r3;
            };
            Vec4 v[4];
            float f[16];
            //data[0]: r0 r1
            //data[1]: r2 r3
            __m256 data[2];
        };
    };
}
#endif