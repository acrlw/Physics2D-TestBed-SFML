#ifndef PHYSICS2D_COMMON_H
#define PHYSICS2D_COMMON_H

#include "cassert"
#include <cmath>
#include <cfloat>
#include <vector>
#include <tuple>
#include <optional>
#include <algorithm>
#include <functional>
#include <memory>
#include <map>
#include <xmmintrin.h>


namespace Physics2D
{
	using real = float;
	namespace Container
	{
		template<class T>
		using Vector = std::vector<T>;

		template<typename K, typename V>
		using Map = std::map<K, V>;

	}
	namespace Constant
	{
		const real Eps = 1e-5f;

		const real HalfPi = std::atan(1.0f) * 2.0f;
		const real Pi = std::atan(1.0f) * 4.0f;
		const real TwoPi = std::atan(1.0f) * 8.0f;
		const real InvPi = 1.0f / Pi;

		const real Rad2Deg = 180.0f / Pi;
		const real Deg2Rad = Pi / 180.0f;

		constexpr real PosInfty = FLT_MAX;
		constexpr real PosZero = FLT_MIN;

		constexpr real NegInfty = -FLT_MAX;
		constexpr real NegZero = -FLT_MIN;

		constexpr unsigned int SimplexMax = 8;
		constexpr unsigned int CCDMaxIterations = 15;
		constexpr real GeometryEpsilon = 1e-5f;
		constexpr real CCDMinVelocity = 100.0f;
		constexpr real MaxVelocity = 1000.0f;
		constexpr real MaxAngularVelocity = 1000.0f;
		constexpr real AABBExpansionFactor = 0.0f;
		constexpr real MinLinearVelocity = 1e-4f;
		constexpr real MinAngularVelocity = 1e-4f;
		constexpr size_t SleepCountdown = 32;

	}
}

#endif
