#ifndef MATH_LINEAR_MATRIX3X3_H
#define MATH_LINEAR_MATRIX3X3_H
#include "../../common/common.h"
#include "../../math/math.h"
#include "../../math/linear/vec3.h"
#include "../../math/linear/vec2.h"
namespace Physics2D
{

    //row first
    struct Mat3
    {
        Mat3();
        Mat3(const Mat3& mat);
        Mat3(const Vec3& r0, const Vec3& r1, const Vec3& r2);
        Mat3(
            const real& e00, const real& e01, const real& e02,
            const real& e10, const real& e11, const real& e12,
            const real& e20, const real& e21, const real& e22);
        Mat3(Mat3&& other) = default;

        Mat3 operator+(const Mat3& rhs);
        Mat3 operator-(const Mat3& rhs);
        Vec3 operator[](uint8_t index);

        Mat3& operator=(const Mat3& rhs);
        Mat3& operator+=(const Mat3& rhs);
        Mat3& operator-=(const Mat3& rhs);
        Mat3& operator*=(const real& factor);
        Mat3& operator/=(const real& factor);

        Vec3 c0()const;
        Vec3 c1()const;
        Vec3 c2()const;

        real e00()const;
        real e01()const;
        real e02()const;

        real e10()const;
        real e11()const;
        real e12()const;

        real e20()const;
        real e21()const;
        real e22()const;

        Mat3& set(
            const real& e00, const real& e01, const real& e02,
            const real& e10, const real& e11, const real& e12,
            const real& e20, const real& e21, const real& e22);
        Mat3& set(const Vec3& r0, const Vec3& r1, const Vec3& r2);
        Mat3& set(const Mat3& other);
        Mat3& clear();

        Vec3 multiply(const Vec3& rhs)const;
        Mat3& multiply(const Mat3& rhs);
        real determinant()const;
        Mat3& transpose();
        Mat3& invert();

        static Mat3 skewSymmetricMatrix(const Vec3& v);
        static Mat3 identityMatrix();
        static Mat3 multiply(const Mat3& lhs, const Mat3& rhs);
        static Vec3 multiply(const Mat3& lhs, const Vec3& rhs);
        static real determinant(const Mat3& mat);
        static Mat3 rotate(const Vec3& axis, const real& deg);
        static bool invert(Mat3& mat);

        union
        {
            struct
            {
                Vec3 r0;
                Vec3 r1;
                Vec3 r2;
            };
            Vec3 v[3];
            float f[12];
        };
    };
}
#endif
