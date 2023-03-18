#ifndef MATH_LINEAR_H
#define MATH_LINEAR_H
#include "physics2d_vec2.h"
#include "physics2d_vec3.h"
#include "physics2d_vec4.h"
#include "physics2d_mat2.h"
#include "physics2d_mat3.h"
#include "physics2d_mat4.h"

namespace Physics2D
{
	struct Transform
	{
		Vec2 position;
		real rotation = 0.0f;

		Vec2 transform(const Vec2& source)const
		{
			return Mat2(rotation).multiply(source) + position;
		}
		Vec2 translate(const Vec2& source)const
		{
			return source + position;
		}
		Vec2 rotate(const Vec2& source)const
		{
			return Mat2(rotation).multiply(source);
		}
	};
	inline Vec2 operator*(const real& f, const Vec2& v)
	{
		return v * f;
	}
	inline Vec3 operator*(const real& f, const Vec3& v)
	{
		return v * f;
	}
}
#endif