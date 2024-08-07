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

	PHYSICS2D_API inline bool fuzzyRealEqual(const real& lhs, const real& rhs,
	                                         const real& epsilon = Constant::GeometryEpsilon)
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
		PHYSICS2D_API static real bernstein(const real& t, const real& i, const real& n)
		{
			return combination(n, i) * std::pow(t, i) * std::pow(1.0f - t, n - i);
		}

		PHYSICS2D_API static real dBernstein(const real& t, const real& i, const real& n)
		{
			if(i == 0)
				return -1.0f * n * std::pow(1.0f - t, n - 1);

			if(i == n)
				return n * std::pow(t, n - 1);

			return combination(n, i) * (i * std::pow(t, i - 1) * std::pow(1.0f - t, n - i) -
			                             (n - i) * std::pow(t, i) * std::pow(1.0f - t, n - i - 1));
		}

		PHYSICS2D_API static real d2Bernstein(const real& t, const real& i, const real& n)
		{
			if(i == 0)
				return n * (n - 1) * std::pow(1.0f - t, n - 2.0f);

			if (i == 1)
				return combination(n, 1) * ( -2.0f * (n - 1.0f) * std::pow(1.0f - t, n - 2.0f) + 
					(n - 1.0f) * (n - 2.0f) * t * std::pow(1.0f - t, n - 3.0f));

			if (i == 2)
				return combination(n, 2) * (2.0f * std::pow(1.0f - t, n - 2.0f) 
					- 4.0f * (n - 2.0f) * t * std::pow(1.0f - t, n - 3.0f) +
					(n - 2.0f) * (n - 3.0f) * t * t * std::pow(1.0f - t, n - 3.0f));

			if(i == n)
				return n * (n - 1) * std::pow(t, n - 2.0f);

			
			return combination(n, i) * (i * (i - 1.0f) * std::pow(t, i - 2.0f) * std::pow(1.0f - t, n - i) 
				- 2.0f * i * (n - i)  * std::pow(t, i - 1.0f) * std::pow(1.0f - t, n - i - 1.0f) +
				(n - i) * (n - i - 1.0f) * std::pow(t, i) * std::pow(1.0f - t, n - i - 2.0f));
		}

		PHYSICS2D_API static real combination(const real& n, const real& m)
		{
			real a = 1.0f, b = 1.0f, c = 1.0f;
			for (real i = n; i > 0; i -= 1.0f)
				a *= i;
			for (real i = m; i > 0; i -= 1.0f)
				b *= i;
			for (real i = n - m; i > 0; i -= 1.0f)
				c *= i;
			return a / (b * c);
		}

		//trigonometric function
		PHYSICS2D_API static real abs(const real& x)
		{
			return std::fabs(x);
		}

		PHYSICS2D_API static real sinx(const real& x)
		{
			return std::sin(x);
		}

		PHYSICS2D_API static real cosx(const real& x)
		{
			return std::cos(x);
		}

		PHYSICS2D_API static real tanx(const real& x)
		{
			return std::tan(x);
		}

		PHYSICS2D_API static real arcsinx(const real& x)
		{
			return std::asin(x);
		}

		PHYSICS2D_API static real arccosx(const real& x)
		{
			return std::acos(x);
		}

		PHYSICS2D_API static real arctanx(const real& y, const real& x)
		{
			return std::atan2(y, x);
		}

		PHYSICS2D_API static real max(const real& a, const real& b)
		{
			return std::max(a, b);
		}

		PHYSICS2D_API static real min(const real& a, const real& b)
		{
			return std::min(a, b);
		}

		PHYSICS2D_API static real tripleMin(const real& a, const real& b, const real& c)
		{
			return std::min(std::min(a, b), c);
		}

		PHYSICS2D_API static real tripleMax(const real& a, const real& b, const real& c)
		{
			return std::max(std::max(a, b), c);
		}

		PHYSICS2D_API static real absMax(const real& a, const real& b)
		{
			return std::max(std::fabs(a), std::fabs(b));
		}

		PHYSICS2D_API static real absMin(const real& a, const real& b)
		{
			return std::min(std::fabs(a), std::fabs(b));
		}

		PHYSICS2D_API static real sqrt(const real& x)
		{
			return std::sqrt(x);
		}

		PHYSICS2D_API static real pow(const real& x, const real& e)
		{
			return std::pow(x, e);
		}

		//other
		PHYSICS2D_API static bool sameSign(const real& a, const real& b)
		{
			return a >= 0 && b >= 0 || a <= 0 && b <= 0;
		}

		PHYSICS2D_API static bool sameSign(const real& a, const real& b, const real& c)
		{
			return a >= 0 && b >= 0 && c >= 0 || a <= 0 && b <= 0 && c <= 0;
		}

		PHYSICS2D_API static bool sameSignStrict(const real& a, const real& b)
		{
			return a > 0 && b > 0 || a < 0 && b < 0;
		}

		PHYSICS2D_API static bool sameSignStrict(const real& a, const real& b, const real& c)
		{
			return a > 0 && b > 0 && c > 0 || a < 0 && b < 0 && c < 0;
		}

		PHYSICS2D_API static int8_t sign(const real& num)
		{
			return num > 0 ? 1 : -1;
		}

		PHYSICS2D_API static bool isInRange(const real& value, const real& low, const real& high,
		                                    const real& epsilon = Constant::GeometryEpsilon)
		{
			return value >= low - epsilon && value <= high + epsilon;
		}

		PHYSICS2D_API static bool fuzzyIsInRange(const real& value, const real& low, const real& high,
		                                         const real& epsilon = Constant::GeometryEpsilon)
		{
			return value >= low - epsilon && value <= high + epsilon || value <= low + epsilon && low >= high - epsilon;
		}

		PHYSICS2D_API static real clamp(const real& num, const real& low, const real& high)
		{
			return std::clamp(num, low, high);
		}

		PHYSICS2D_API static size_t clamp(const size_t& num, const size_t& low, const size_t& high)
		{
			if (num < low)
				return low;
			if (num > high)
				return high;
			return num;
		}

		PHYSICS2D_API static real degreeToRadian(const real& angle)
		{
			return angle * (Constant::Pi / 180.0f);
		}

		PHYSICS2D_API static real radianToDegree(const real& radian)
		{
			return radian * (180.0f / Constant::Pi);
		}

		template <typename T, size_t iterations = 4>
		PHYSICS2D_API static T fastInverseSqrt(T x)
		{
			using Tint = std::conditional_t<sizeof(T) == 8, std::int64_t, std::int32_t>;
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
