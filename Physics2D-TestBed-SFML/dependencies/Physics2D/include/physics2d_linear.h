#ifndef MATH_LINEAR_H
#define MATH_LINEAR_H
#include "physics2d_vector2.h"
#include "physics2d_vector3.h"
#include "physics2d_vector4.h"
#include "physics2d_matrix2x2.h"
#include "physics2d_matrix3x3.h"
#include "physics2d_matrix4x4.h"

namespace Physics2D
{
	inline Vector2 operator*(const real& f, const Vector2& v)
	{
		return v * f;
	}
	inline Vector3 operator*(const real& f, const Vector3& v)
	{
		return v * f;
	}
}
#endif