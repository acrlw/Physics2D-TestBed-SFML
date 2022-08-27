#ifndef PHYSICS2D_CLIP_H
#define PHYSICS2D_CLIP_H
#include "../../common/common.h"
#include "../../geometry/shape.h"
#include "../../math/linear/linear.h"
#include "gjk.h"
namespace Physics2D
{
	class ContactGenerator
	{
	public:
		struct ClipEdge
		{
			Vec2 p1;
			Vec2 p2;
			Vec2 normal;
			bool isEmpty()const
			{
				return p1.isOrigin() && p2.isOrigin();
			}
		};
		static Container::Vector<Vec2> dumpVertices(const Transform& transform, Shape* shape);
		static ClipEdge findClipEdge(const Container::Vector<Vec2>& vertices, size_t index, const Vec2& normal);
		static ClipEdge dumpClipEdge(const Transform& transform, Shape* shape, const Container::Vector<Vec2>& vertices, const Vec2& normal);
		static std::pair<ClipEdge, ClipEdge> recognize(const Transform& transformA, Shape* shapeA, const Transform& transformB, Shape* shapeB, const Vec2& normal);
		static Container::Vector<PointPair> clip(const ClipEdge& clipEdgeA, const ClipEdge& clipEdgeB, const Vec2& normal);
	};
}
#endif
