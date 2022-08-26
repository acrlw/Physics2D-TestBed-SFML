#ifndef PHYSICS2D_BROADPHASE_BOUNDINGSPHERE_H
#define PHYSICS2D_BROADPHASE_BOUNDINGSPHERE_H

#include "../../math/linear/linear.h"
#include "../../geometry/shape.h"

namespace Physics2D
{
	struct BoundingSphere
	{

		real radius;
		Vec2 position;
	};
}
#endif