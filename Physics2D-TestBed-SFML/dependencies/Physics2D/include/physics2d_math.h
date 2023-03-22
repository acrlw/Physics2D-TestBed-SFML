#ifndef PHYSICS2D_MATH_H
#define PHYSICS2D_MATH_H
#include "physics2d_common.h"
#include "immintrin.h"
#include "xmmintrin.h"
namespace Physics2D
{

	//basic real utilities
	PHYSICS2D_API inline void realSwap(real& lhs, real& rhs)
	{
		const real temp = lhs;
		lhs = rhs;
		rhs = temp;
	}
	PHYSICS2D_API inline bool fuzzyRealEqual(const real& lhs, const real& rhs, const real& epsilon = Constant::GeometryEpsilon)
	{
		return std::fabs(lhs - rhs) < epsilon;
	}
	PHYSICS2D_API inline bool realEqual(const real& lhs, const real& rhs)
	{
		return fuzzyRealEqual(lhs, rhs, Constant::GeometryEpsilon);
	}
	class Math
	{
	public:
		//trigonometric function
		PHYSICS2D_API static inline real abs(const real& x)
		{
			return std::fabs(x);
		}
		PHYSICS2D_API static inline real sinx(const real& x)
		{
			return std::sin(x);
		}
		PHYSICS2D_API static inline real cosx(const real& x)
		{
			return std::cos(x);
		}
		PHYSICS2D_API static inline real tanx(const real& x)
		{
			return std::tan(x);
		}
		PHYSICS2D_API static inline real arcsinx(const real& x)
		{
			return std::asin(x);
		}
		PHYSICS2D_API static inline real arccosx(const real& x)
		{
			return std::acos(x);
		}
		PHYSICS2D_API static inline real arctanx(const real& y,const real& x)
		{
			return std::atan2(y, x);
		}
		PHYSICS2D_API static inline real max(const real& a, const real& b)
		{
			return std::max(a, b);
		}
		PHYSICS2D_API static inline real min(const real& a, const real& b)
		{
			return std::min(a, b);
		}
		PHYSICS2D_API static inline real tripleMin(const real& a, const real& b, const real& c)
		{
			return std::min(std::min(a, b), c);
		}
		PHYSICS2D_API static inline real tripleMax(const real& a, const real& b, const real& c)
		{
			return std::max(std::max(a, b), c);
		}
		PHYSICS2D_API static inline real absMax(const real& a, const real& b)
		{
			return std::max(std::fabs(a), std::fabs(b));
		}
		PHYSICS2D_API static inline real absMin(const real& a, const real& b)
		{
			return std::min(std::fabs(a), std::fabs(b));
		}
		PHYSICS2D_API static inline real sqrt(const real& x)
		{
			return std::sqrt(x);
		}
		PHYSICS2D_API static inline real pow(const real& x, const real& e)
		{
			return std::pow(x, e);
		}
		//other
		PHYSICS2D_API static inline bool sameSign(const real& a, const real& b)
		{
			return a >= 0 && b >= 0 || a <= 0 && b <= 0;
		}
		PHYSICS2D_API static inline bool sameSign(const real& a, const real& b, const real& c)
		{
			return a >= 0 && b >= 0 && c >= 0 || a <= 0 && b <= 0 && c <= 0;
		}
		PHYSICS2D_API static inline int8_t sign(const real& num)
		{
			return num > 0 ? 1 : -1;
		}
		PHYSICS2D_API static inline bool isInRange(const real& value, const real& low, const real& high, const real& epsilon = Constant::GeometryEpsilon)
		{
			return value >= low - epsilon && value <= high + epsilon;
		}
		PHYSICS2D_API static inline bool fuzzyIsInRange(const real& value, const real& low, const real& high, const real& epsilon = Constant::GeometryEpsilon)
		{
			return value >= low - epsilon && value <= high + epsilon || value <= low + epsilon && low >= high - epsilon;
		}
		PHYSICS2D_API static inline real clamp(const real& num, const real& low, const real& high)
		{
			return std::clamp(num, low, high);
		}
		PHYSICS2D_API static inline size_t clamp(const size_t& num, const size_t& low, const size_t& high)
		{
			if (num < low)
				return low;
			if (num > high)
				return high;
			return num;
		}
		PHYSICS2D_API static inline real degreeToRadian(const real& angle)
		{
			return angle * (Constant::Pi / 180.0f);
		}
		PHYSICS2D_API static inline real radianToDegree(const real& radian)
		{
			return radian * (180.0f /Constant::Pi);
		}
		template <typename T, size_t iterations = 3> PHYSICS2D_API static T fastInverseSqrt(T x) {
			typedef typename std::conditional<sizeof(T) == 8, std::int64_t, std::int32_t>::type Tint;
			T y = x;
			T x2 = y * 0.5f;
			Tint i = *(Tint*)&y;
			i = (sizeof(T) == 8 ? 0x5fe6eb50c7b537a9 : 0x5f3759df) - (i >> 1);
			y = *(T*)&i;
			for (size_t k = 0; k <= iterations; k++)
				y = y * (1.5f - (x2 * y * y));
			return y;
		}
	};
}
#endif