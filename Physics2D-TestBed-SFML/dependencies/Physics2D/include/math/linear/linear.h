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