#ifndef MATH_LINEAR_QUATERNION_H
#define MATH_LINEAR_QUATERNION_H
#include "physics2d_common.h"
#include "physics2d_vector4.h"
namespace Physics2D
{
	struct PHYSICS2D_API Quaternion
	{
		Quaternion(const real& s, const real& i, const real& j, const real& k);
		Quaternion(const Vector4& vec4);
		Quaternion(const real& s, const Vector3& vec3);
		Quaternion(const Quaternion& copy) = default;
		Quaternion(Quaternion&& copy) = default;
		real s;
		Vector3 v;
	};
}
#endif