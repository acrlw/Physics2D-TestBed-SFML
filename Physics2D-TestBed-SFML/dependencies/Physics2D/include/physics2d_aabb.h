#ifndef PHYSICS2D_BROADPHASE_AABB_H
#define PHYSICS2D_BROADPHASE_AABB_H

#include "physics2d_linear.h"
#include "physics2d_shape.h"

namespace Physics2D
{
	class Body;

	struct AABB
	{
		AABB() = default;
		AABB(const Vec2& topLeft, const real& boxWidth, const real& boxHeight);
		AABB(const Vec2& topLeft, const Vec2& bottomRight);
		real width = 0;
		real height = 0;
		Vec2 position;
		inline Vec2 topLeft()const;
		inline Vec2 topRight()const;
		inline Vec2 bottomLeft()const;
		inline Vec2 bottomRight()const;

		inline real minimumX()const;
		inline real minimumY()const;
		inline real maximumX()const;
		inline real maximumY()const;

		bool collide(const AABB& other) const;
		void expand(const real& factor);
		void scale(const real& factor);
		void clear();
		AABB& unite(const AABB& other);
		real surfaceArea()const;
		real volume()const;
		bool isSubset(const AABB& other)const;
		bool isEmpty()const;
		bool operator==(const AABB& other)const;
		bool raycast(const Vec2& start, const Vec2& direction)const;
		/// <summary>
		/// Create AABB from shape.
		/// </summary>
		/// <param name="shape">shape source</param>
		/// <param name="factor">AABB scale factor. Default factor 1 means making tight AABB</param>
		/// <returns></returns>
		static AABB fromShape(const Transform& transform, Shape* shape, const real& factor = 0);
		static AABB fromBody(Body* body, const real& factor = 0);
		static AABB fromBox(const Vec2& topLeft, const Vec2& bottomRight);
		/// <summary>
		/// Check if two aabbs are overlapping
		/// </summary>
		/// <param name="src"></param>
		/// <param name="target"></param>
		/// <returns></returns>
		static bool collide(const AABB& src, const AABB& target);
		/// <summary>
		/// Return two aabb union result
		/// </summary>
		/// <param name="src"></param>
		/// <param name="target"></param>
		/// <returns></returns>
		static AABB unite(const AABB& src, const AABB& target, const real& factor = 0);
		/// <summary>
		/// Check if b is subset of a
		/// </summary>
		/// <param name="a"></param>
		/// <param name="b"></param>
		/// <returns></returns>
		static bool isSubset(const AABB& a, const AABB& b);

		static void expand(AABB& aabb, const real& factor = 0.0);

		static bool raycast(const AABB& aabb, const Vec2& start, const Vec2& direction);
		
	};


}
#endif