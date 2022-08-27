#ifndef MATH_LINEAR_H
#define MATH_LINEAR_H
#include "../../math/linear/vec2.h"
#include "../../math/linear/vec3.h"
#include "../../math/linear/vec4.h"
#include "../../math/linear/mat2.h"
#include "../../math/linear/mat3.h"
#include "../../math/linear/mat4.h"

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