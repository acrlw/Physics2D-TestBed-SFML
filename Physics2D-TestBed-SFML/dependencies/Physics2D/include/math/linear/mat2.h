#ifndef MATH_LINEAR_Mat2_H
#define MATH_LINEAR_Mat2_H
#include "../../math/linear/vec2.h"
#include "../../math/math.h"
namespace Physics2D
{

    //row first
    struct alignas(16) Mat2
    {
        Mat2();
        Mat2(const real& radian);
        Mat2(const Mat2& mat);
        Mat2(const Vec2& r0, const Vec2& r1);
        Mat2(
            const real& e00, const real& e01,
            const real& e10, const real& e11);
        Mat2(__m128 d);
        Mat2(Mat2&& other) = default;

        Mat2& operator=(const Mat2& rhs);
        Mat2& operator+=(const Mat2& rhs);
        Mat2& operator-=(const Mat2& rhs);
        Mat2& operator*=(const real& factor);
        Mat2& operator/=(const real& factor);
        Mat2 operator+(const Mat2& rhs)const;
        Mat2 operator-(const Mat2& rhs)const;
        Vec2& operator[](uint8_t index);


        Vec2 c0()const;
        Vec2 c1()const;

        real e00()const;
        real e01()const;

        real e10()const;
        real e11()const;

        real determinant()const;
        Mat2& transpose();
        Mat2& invert();
        Mat2& multiply(const Mat2& rhs);
        Vec2 multiply(const Vec2& rhs)const;

        Mat2& clear();
        Mat2& set(
            const real& e00, const real& e01,
            const real& e10, const real& e11);
        Mat2& set(const Vec2& c0, const Vec2& c1);
        Mat2& set(const Mat2& other);
        Mat2& set(const real& radian);

        Mat2& swap(Mat2& other);

        static Mat2 skewSymmetricMatrix(const Vec2& r);
        static Mat2 identity();
        static Vec2 multiply(const Mat2& lhs, const Vec2& rhs);
        static Mat2 multiply(const Mat2& lhs, const Mat2& rhs);
        static real determinant(const Mat2& mat);
        static bool invert(Mat2& mat);

        union
        {
            float f[4]; //0 1 2 3 --> x0 y0 x1 y1
            //[x0 y0]
            //[x1 y1]
            struct
            {
                Vec2 r0;
                Vec2 r1;
            };
            Vec2 v[2];
            __m128 data;
        };
    };
}
#endif
