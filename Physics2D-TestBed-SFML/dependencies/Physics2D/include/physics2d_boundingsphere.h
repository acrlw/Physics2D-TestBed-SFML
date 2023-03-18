#ifndef PHYSICS2D_BROADPHASE_BOUNDINGSPHERE_H
#define PHYSICS2D_BROADPHASE_BOUNDINGSPHERE_H

#include "physics2d_linear.h"
#include "physics2d_shape.h"

namespace Physics2D
{
	struct BoundingSphere
	{

		real radius;
		Vec2 position;
	};
}
#endif