#ifndef PHYSICS2D_SCENES_SOLVER_H
#define PHYSICS2D_SCENES_SOLVER_H
#include "frame.h"

namespace Physics2D
{

	class RationalCubicBezier
	{
	public:

		void set(const Vector2& p0, const Vector2& p1, const Vector2& p2, const Vector2& p3, float w0 = 1.0f, float w1 = 1.0f, float w2 = 1.0f, float w3 = 1.0f)
		{
			m_points[0] = p0;
			m_points[1] = p1;
			m_points[2] = p2;
			m_points[3] = p3;
			m_weights[0] = w0;
			m_weights[1] = w1;
			m_weights[2] = w2;
			m_weights[3] = w3;
		}

		Vector2& pointAt(size_t index)
		{
			return m_points[index];
		}

		float& weightAt(size_t index)
		{
			return m_weights[index];
		}

		Vector2 sample(float t)const
		{
			float u0 = m_weights[0] * Math::bernstein(t, 0.0f, 3.0f);
			float u1 = m_weights[1] * Math::bernstein(t, 1.0f, 3.0f);
			float u2 = m_weights[2] * Math::bernstein(t, 2.0f, 3.0f);
			float u3 = m_weights[3] * Math::bernstein(t, 3.0f, 3.0f);

			Vector2 result = (m_points[0] * u0 + m_points[1] * u1 + m_points[2] * u2 + m_points[3] * u3) / (u0 + u1 + u2 + u3);

			return result;
		}

		std::vector<Vector2> samplePoints(size_t count)const
		{
			std::vector<Vector2> result;
			result.reserve(count);
			float step = 1.0f / static_cast<float>(count);

			for (float t = 0.0f; t <= 1.0f; t += step)
				result.push_back(sample(t));

			return result;
		}

		float curvature(float t)const
		{
			// \left( (t-1)^3w_0-t\left( 3(t-1)^2w_1+t\left( tw_3-3(t-1)w_2 \right) \right) \right) ^2
			float det = std::pow((std::pow(t - 1.0f, 3.0f) * m_weights[0] -
				t * (3.0f * std::pow(t - 1.0f, 2.0f) * m_weights[1] + t * (t * m_weights[3] - 3.0f * (t - 1.0f) * m_weights[2]))), 2.0f);

			// -3(t-1)^2w_0\left( (t-1)^2w_1+t\left( tw_3-2(t-1)w_2 \right) \right)

			float dp0 = -3.0f * std::pow(t - 1.0f, 2.0f) * m_weights[0] *
				(std::pow(t - 1.0f, 2.0f) * m_weights[1] + t * (t * m_weights[3] - 2.0f * (t - 1.0f) * m_weights[2]));

			// +3(t-1)w_1\left( t^2\left( 2tw_3-3(t-1)w_2 \right) +(t-1)^3w_0 \right)

			float dp1 = 3.0f * (t - 1.0f) * m_weights[1] *
				(t * t * (2.0f * t * m_weights[3] - 3.0f * (t - 1.0f) * m_weights[2]) + std::pow(t - 1.0f, 3.0f) * m_weights[0]);

			// -3tw_2\left( t\left( t^2w_3-3(t-1)^2w_1 \right) +2(t-1)^3w_0 \right)

			float dp2 = -3.0f * t * m_weights[2] *
				(t * (t * t * m_weights[3] - 3.0f * std::pow(t - 1.0f, 2.0f) * m_weights[1]) + 2.0f * std::pow(t - 1.0f, 3.0f) * m_weights[0]);

			// +3t^2w_3\left( (t-1)^2w_0+t\left( tw_2-2(t-1)w_1 \right) \right)

			float dp3 = 3.0f * t * t * m_weights[3] *
				(std::pow(t - 1.0f, 2.0f) * m_weights[0] + t * (t * m_weights[2] - 2.0f * (t - 1.0f) * m_weights[1]));

			Vector2 dp = dp0 * m_points[0] + dp1 * m_points[1] + dp2 * m_points[2] + dp3 * m_points[3];
			dp /= det;

			// 6(t-1)w_0\left( (t-1)^3w_0\left( \left( -2t^2+t+1 \right) w_2+(t-1)^2w_1+t(t+1)w_3 \right) +t^2\left( t\left( 5t^2-13t+8 \right) w_3w_2-(t-2)t^2w_{3}^{2}-3(t-1)^2(2t-3)w_{2}^{2} \right) -3(t-1)^5w_{1}^{2}+t(t-1)^2w_1\left( 9(t-1)^2w_2+t(3-4t)w_3 \right) \right)

			float ddp0 = 6.0f * (t - 1.0f) * m_weights[0] *
				(std::pow(t - 1.0f, 3.0f) * m_weights[0] * ((-2.0f * t * t + t + 1.0f) * m_weights[2] + std::pow(t - 1.0f, 2.0f) * m_weights[1] + t * (t + 1.0f) * m_weights[3]) + t * t * (t * (5.0f * t * t - 13.0f * t + 8.0f) * m_weights[3] * m_weights[2] - (t - 2.0f) * t * t * m_weights[3] * m_weights[3] - 3.0f * std::pow(t - 1.0f, 2.0f) * (2.0f * t - 3.0f) * m_weights[2] * m_weights[2]) - 3.0f * std::pow(t - 1.0f, 5.0f) * m_weights[1] * m_weights[1] + t * std::pow(t - 1.0f, 2.0f) * m_weights[1] * (9.0f * std::pow(t - 1.0f, 2.0f) * m_weights[2] + t * (3.0f - 4.0f * t) * m_weights[3]));

			// -6w_1\left( t^3\left( t^2(3-2t)w_{3}^{2}-9(t-1)^3w_{2}^{2}+9t(t-1)^2w_2w_3+3(t-1)^2w_1\left( 3(t-1)w_2-(2t+1)w_3 \right) \right) +(t-1)^6w_{0}^{2}+(t-1)^3w_0\left( t\left( t(t+6)w_3-9(t-1)w_2 \right) -3(t-1)^3w_1 \right) \right) 

			float ddp1 = -6.0f * m_weights[1] *
				(t * t * t * (t * t * (3.0f - 2.0f * t) * m_weights[3] * m_weights[3] - 9.0f * std::pow(t - 1.0f, 3.0f) * m_weights[2] * m_weights[2] + 9.0f * t * std::pow(t - 1.0f, 2.0f) * m_weights[2] * m_weights[3] + 3.0f * std::pow(t - 1.0f, 2.0f) * m_weights[1] * (3.0f * (t - 1.0f) * m_weights[2] - (2.0f * t + 1.0f) * m_weights[3])) + std::pow(t - 1.0f, 6.0f) * m_weights[0] * m_weights[0] + std::pow(t - 1.0f, 3.0f) * m_weights[0] * (t * (t * (t + 6.0f) * m_weights[3] - 9.0f * (t - 1.0f) * m_weights[2]) - 3.0f * std::pow(t - 1.0f, 3.0f) * m_weights[1]));

			// 6w_2\left( t^3\left( t^3\left( 3w_2-w_3 \right) w_3+9(t-1)^3w_{1}^{2}-9(t-1)w_1\left( (t-1)^2w_2+tw_3 \right) \right) -t^2(t-1)^2w_0\left( -3\left( 2t^2-5t+3 \right) w_2+9(t-1)^2w_1+(t-7)tw_3 \right) +(2t+1)(t-1)^5w_{0}^{2} \right) 

			float ddp2 = 6.0f * m_weights[2] *
				(t * t * t * (t * t * t * (3.0f * m_weights[2] - m_weights[3]) * m_weights[3] + 9.0f * std::pow(t - 1.0f, 3.0f) * m_weights[1] * m_weights[1] - 9.0f * (t - 1.0f) * m_weights[1] * (std::pow(t - 1.0f, 2.0f) * m_weights[2] + t * m_weights[3])) - t * t * (t - 1.0f) * (t - 1.0f) * m_weights[0] * (-3.0f * (2.0f * t * t - 5.0f * t + 3.0f) * m_weights[2] + 9.0f * std::pow(t - 1.0f, 2.0f) * m_weights[1] + (t - 7.0f) * t * m_weights[3]) + (2.0f * t + 1.0f) * std::pow(t - 1.0f, 5.0f) * m_weights[0] * m_weights[0]);

			// -6tw_3\left( t^5w_2\left( 3w_2-w_3 \right) +t^4w_1\left( (2t-3)w_3-9(t-1)w_2 \right) +3(t-1)^2(2t+1)t^2w_{1}^{2}-(t-1)tw_0\left( t\left( \left( -4t^2+5t-1 \right) w_2+(t-2)tw_3 \right) +(5t+3)(t-1)^2w_1 \right) +(t-1)^4(t+1)w_{0}^{2} \right) 

			float ddp3 = -6.0f * t * m_weights[3] *
				(std::pow(t, 5.0f) * m_weights[2] * (3.0f * m_weights[2] - m_weights[3]) + std::pow(t, 4.0f) * m_weights[1] * ((2.0f * t - 3.0f) * m_weights[3] - 9.0f * (t - 1.0f) * m_weights[2]) + 3.0f * std::pow(t - 1.0f, 2.0f) * (2.0f * t + 1.0f) * t * t * m_weights[1] * m_weights[1] - (t - 1.0f) * t * m_weights[0] * (t * ((-4.0f * t * t + 5.0f * t - 1.0f) * m_weights[2] + (t - 2.0f) * t * m_weights[3]) + (5.0f * t + 3.0f) * std::pow(t - 1.0f, 2.0f) * m_weights[1]) + std::pow(t - 1.0f, 4.0f) * (t + 1.0f) * m_weights[0] * m_weights[0]);

			Vector2 ddp = ddp0 * m_points[0] + ddp1 * m_points[1] + ddp2 * m_points[2] + ddp3 * m_points[3];

			ddp /= det;

			float k = std::abs(dp.x * ddp.y - dp.y * ddp.x) / std::pow(dp.length(), 3.0f);

			return k;
		}

		std::vector<Vector2> curvaturePoints(size_t count, float scale = 1.0, bool flip = false) const
		{
			std::vector<Vector2> result;
			result.reserve(count);

			float step = 1.0f / static_cast<float>(count);
			for (float t = 0.0f; t <= 1.0f; t += step)
			{
				Vector2 p1 = sample(t);


				// \left( (t-1)^3w_0-t\left( 3(t-1)^2w_1+t\left( tw_3-3(t-1)w_2 \right) \right) \right) ^2
				float det = std::pow((std::pow(t - 1.0f, 3.0f) * m_weights[0] - 
					t * (3.0f * std::pow(t - 1.0f, 2.0f) * m_weights[1] + t * (t * m_weights[3] - 3.0f * (t - 1.0f) * m_weights[2]))), 2.0f);

				// -3(t-1)^2w_0\left( (t-1)^2w_1+t\left( tw_3-2(t-1)w_2 \right) \right)

				float dp0 = -3.0f * std::pow(t - 1.0f, 2.0f) * m_weights[0] * 
					(std::pow(t - 1.0f, 2.0f) * m_weights[1] + t * (t * m_weights[3] - 2.0f * (t - 1.0f) * m_weights[2]));

				// +3(t-1)w_1\left( t^2\left( 2tw_3-3(t-1)w_2 \right) +(t-1)^3w_0 \right)

				float dp1 = 3.0f * (t - 1.0f) * m_weights[1] * 
					(t * t * (2.0f * t * m_weights[3] - 3.0f * (t - 1.0f) * m_weights[2]) + std::pow(t - 1.0f, 3.0f) * m_weights[0]);

				// -3tw_2\left( t\left( t^2w_3-3(t-1)^2w_1 \right) +2(t-1)^3w_0 \right)

				float dp2 = -3.0f * t * m_weights[2] * 
					(t * (t * t * m_weights[3] - 3.0f * std::pow(t - 1.0f, 2.0f) * m_weights[1]) + 2.0f * std::pow(t - 1.0f, 3.0f) * m_weights[0]);

				// +3t^2w_3\left( (t-1)^2w_0+t\left( tw_2-2(t-1)w_1 \right) \right)

				float dp3 = 3.0f * t * t * m_weights[3] * 
					(std::pow(t - 1.0f, 2.0f) * m_weights[0] + t * (t * m_weights[2] - 2.0f * (t - 1.0f) * m_weights[1]));

				Vector2 dp = dp0 * m_points[0] + dp1 * m_points[1] + dp2 * m_points[2] + dp3 * m_points[3];
				dp /= det;

				// 6(t-1)w_0\left( (t-1)^3w_0\left( \left( -2t^2+t+1 \right) w_2+(t-1)^2w_1+t(t+1)w_3 \right) +t^2\left( t\left( 5t^2-13t+8 \right) w_3w_2-(t-2)t^2w_{3}^{2}-3(t-1)^2(2t-3)w_{2}^{2} \right) -3(t-1)^5w_{1}^{2}+t(t-1)^2w_1\left( 9(t-1)^2w_2+t(3-4t)w_3 \right) \right)

				float ddp0 = 6.0f * (t - 1.0f) * m_weights[0] * 
					(std::pow(t - 1.0f, 3.0f) * m_weights[0] * ((-2.0f * t * t + t + 1.0f) * m_weights[2] + std::pow(t - 1.0f, 2.0f) * m_weights[1] + t * (t + 1.0f) * m_weights[3]) + t * t * (t * (5.0f * t * t - 13.0f * t + 8.0f) * m_weights[3] * m_weights[2] - (t - 2.0f) * t * t * m_weights[3] * m_weights[3] - 3.0f * std::pow(t - 1.0f, 2.0f) * (2.0f * t - 3.0f) * m_weights[2] * m_weights[2]) - 3.0f * std::pow(t - 1.0f, 5.0f) * m_weights[1] * m_weights[1] + t * std::pow(t - 1.0f, 2.0f) * m_weights[1] * (9.0f * std::pow(t - 1.0f, 2.0f) * m_weights[2] + t * (3.0f - 4.0f * t) * m_weights[3]));

				// -6w_1\left( t^3\left( t^2(3-2t)w_{3}^{2}-9(t-1)^3w_{2}^{2}+9t(t-1)^2w_2w_3+3(t-1)^2w_1\left( 3(t-1)w_2-(2t+1)w_3 \right) \right) +(t-1)^6w_{0}^{2}+(t-1)^3w_0\left( t\left( t(t+6)w_3-9(t-1)w_2 \right) -3(t-1)^3w_1 \right) \right) 

				float ddp1 = -6.0f * m_weights[1] * 
					(t * t * t * (t * t * (3.0f - 2.0f * t) * m_weights[3] * m_weights[3] - 9.0f * std::pow(t - 1.0f, 3.0f) * m_weights[2] * m_weights[2] + 9.0f * t * std::pow(t - 1.0f, 2.0f) * m_weights[2] * m_weights[3] + 3.0f * std::pow(t - 1.0f, 2.0f) * m_weights[1] * (3.0f * (t - 1.0f) * m_weights[2] - (2.0f * t + 1.0f) * m_weights[3])) + std::pow(t - 1.0f, 6.0f) * m_weights[0] * m_weights[0] + std::pow(t - 1.0f, 3.0f) * m_weights[0] * (t * (t * (t + 6.0f) * m_weights[3] - 9.0f * (t - 1.0f) * m_weights[2]) - 3.0f * std::pow(t - 1.0f, 3.0f) * m_weights[1]));

				// 6w_2\left( t^3\left( t^3\left( 3w_2-w_3 \right) w_3+9(t-1)^3w_{1}^{2}-9(t-1)w_1\left( (t-1)^2w_2+tw_3 \right) \right) -t^2(t-1)^2w_0\left( -3\left( 2t^2-5t+3 \right) w_2+9(t-1)^2w_1+(t-7)tw_3 \right) +(2t+1)(t-1)^5w_{0}^{2} \right) 

				float ddp2 = 6.0f * m_weights[2] * 
					(t * t * t * (t * t * t * (3.0f * m_weights[2] - m_weights[3]) * m_weights[3] + 9.0f * std::pow(t - 1.0f, 3.0f) * m_weights[1] * m_weights[1] - 9.0f * (t - 1.0f) * m_weights[1] * (std::pow(t - 1.0f, 2.0f) * m_weights[2] + t * m_weights[3])) - t * t * (t - 1.0f) * (t - 1.0f) * m_weights[0] * (-3.0f * (2.0f * t * t - 5.0f * t + 3.0f) * m_weights[2] + 9.0f * std::pow(t - 1.0f, 2.0f) * m_weights[1] + (t - 7.0f) * t * m_weights[3]) + (2.0f * t + 1.0f) * std::pow(t - 1.0f, 5.0f) * m_weights[0] * m_weights[0]);

				// -6tw_3\left( t^5w_2\left( 3w_2-w_3 \right) +t^4w_1\left( (2t-3)w_3-9(t-1)w_2 \right) +3(t-1)^2(2t+1)t^2w_{1}^{2}-(t-1)tw_0\left( t\left( \left( -4t^2+5t-1 \right) w_2+(t-2)tw_3 \right) +(5t+3)(t-1)^2w_1 \right) +(t-1)^4(t+1)w_{0}^{2} \right) 

				float ddp3 = -6.0f * t * m_weights[3] * 
					(std::pow(t, 5.0f) * m_weights[2] * (3.0f * m_weights[2] - m_weights[3]) + std::pow(t, 4.0f) * m_weights[1] * ((2.0f * t - 3.0f) * m_weights[3] - 9.0f * (t - 1.0f) * m_weights[2]) + 3.0f * std::pow(t - 1.0f, 2.0f) * (2.0f * t + 1.0f) * t * t * m_weights[1] * m_weights[1] - (t - 1.0f) * t * m_weights[0] * (t * ((-4.0f * t * t + 5.0f * t - 1.0f) * m_weights[2] + (t - 2.0f) * t * m_weights[3]) + (5.0f * t + 3.0f) * std::pow(t - 1.0f, 2.0f) * m_weights[1]) + std::pow(t - 1.0f, 4.0f) * (t + 1.0f) * m_weights[0] * m_weights[0]);

				Vector2 ddp = ddp0 * m_points[0] + ddp1 * m_points[1] + ddp2 * m_points[2] + ddp3 * m_points[3];

				ddp /= det;

				float k = std::abs(dp.x * ddp.y - dp.y * ddp.x) / std::pow(dp.length(), 3.0f);

				Vector2 tangent = dp.normal();
				Vector2 normal = tangent.perpendicular();
				Vector2 v = normal * k * scale;

				if (flip)
					v.negate();

				Vector2 curvaturePoint = v + p1;
				result.push_back(curvaturePoint);

			}

			return result;
		}

	private:
		std::array<Vector2, 4> m_points;
		//std::array<float, 4> m_weights = {0.45f, 1.0f, 1.0f, 1.0f};
		//std::array<float, 4> m_weights = {1.0f, 1.0f, 1.0f, 1.0f};
		std::array<float, 4> m_weights = {1.0051876f, 0.9668550f, 0.8220000f, 0.8360001f };
	};

	class RationalQuadraticBezier
	{

	public:

		void set(const Vector2& p0, const Vector2& p1, const Vector2& p2, float w0 = 1.0f, float w1 = 1.0f, float w2 = 1.0f)
		{
			m_points[0] = p0;
			m_points[1] = p1;
			m_points[2] = p2;
			m_weights[0] = w0;
			m_weights[1] = w1;
			m_weights[2] = w2;
		}

		Vector2& pointAt(size_t index)
		{
			return m_points[index];
		}

		float& weightAt(size_t index)
		{
			return m_weights[index];
		}

		Vector2 sample(float t)const
		{
			Vector2 result;

			return result;
		}

	private:
		std::array<Vector2, 3> m_points;
		std::array<float, 3> m_weights;
	};

	class QuinticBezier
	{
	public:
		static QuinticBezier fromPoints(const Vector2& p0, const Vector2& p1, const Vector2& p2, const Vector2& p3, const Vector2& p4, const Vector2& p5)
		{
			QuinticBezier bezier;
			bezier.setPoints(p0, p1, p2, p3, p4, p5);
			return bezier;
		}

		Vector2& operator[](size_t index)
		{
			return m_points[index];
		}

		Vector2& at(size_t index)
		{
			return m_points[index];
		}

		void setPoints(const Vector2& p0, const Vector2& p1, const Vector2& p2, const Vector2& p3, const Vector2& p4, const Vector2& p5)
		{
			m_points[0] = p0;
			m_points[1] = p1;
			m_points[2] = p2;
			m_points[3] = p3;
			m_points[4] = p4;
			m_points[5] = p5;
		}

		Vector2 sample(float t)const
		{
			return Math::bernstein(t, 0, 5) * m_points[0] +
				Math::bernstein(t, 1, 5) * m_points[1] +
				Math::bernstein(t, 2, 5) * m_points[2] +
				Math::bernstein(t, 3, 5) * m_points[3] +
				Math::bernstein(t, 4, 5) * m_points[4] +
				Math::bernstein(t, 5, 5) * m_points[5];
		}

		std::vector<Vector2> samplePoints(size_t count)const
		{
			std::vector<Vector2> result;
			result.reserve(count);
			float step = 1.0f / static_cast<float>(count);

			for (float t = 0.0f; t <= 1.0f; t += step)
				result.push_back(sample(t));

			return result;
		}

		std::vector<Vector2> curvaturePoints(size_t count, float scale = 1.0, bool flip = false) const
		{
			std::vector<Vector2> result;
			result.reserve(count);

			float step = 1.0f / static_cast<float>(count);
			for (float t = 0.0f; t <= 1.0f; t += step)
			{

				Vector2 p1 = sample(t);

				//Vector2 dp = Math::dBernstein(t, 0, 5) * m_points[0] +
				//	Math::dBernstein(t, 1, 5) * m_points[1] +
				//	Math::dBernstein(t, 2, 5) * m_points[2] +
				//	Math::dBernstein(t, 3, 5) * m_points[3] +
				//	Math::dBernstein(t, 4, 5) * m_points[4] +
				//	Math::dBernstein(t, 5, 5) * m_points[5];

				//Vector2 ddp = Math::d2Bernstein(t, 0, 5) * m_points[0] +
				//	Math::d2Bernstein(t, 1, 5) * m_points[1] +
				//	Math::d2Bernstein(t, 2, 5) * m_points[2] +
				//	Math::d2Bernstein(t, 3, 5) * m_points[3] +
				//	Math::d2Bernstein(t, 4, 5) * m_points[4] +
				//	Math::d2Bernstein(t, 5, 5) * m_points[5];

				Vector2 dp = -5.0f * std::pow(t - 1.0f, 4.0f) * m_points[0] +
					5.0f * (5.0f * t - 1.0f) * std::pow(t - 1.0f, 3.0f) * m_points[1] +
					-10.0f * t * (5.0f * t - 2.0f) * (t - 1.0f) * (t - 1.0f) * m_points[2] +
					10.0f * t * t * (5.0f * t * t - 8.0f * t + 3.0f) * m_points[3] +
					5.0f * std::pow(t, 3.0f) * (4.0f - 5.0f * t) * m_points[4] +
					5.0f * std::pow(t, 4.0f) * m_points[5];

				Vector2 ddp = -20.0f * std::pow(t - 1.0f, 3.0f) * m_points[0] +
					20.0f * (5.0f * t - 2.0f) * std::pow(t - 1.0f, 2.0f) * m_points[1] +
					-20.0f * (10.0f * std::pow(t, 3.0f) - 18.0f * t * t + 9.0f * t - 1.0f) * m_points[2] +
					20.0f * t * (10.0f * t * t - 12.0f * t + 3.0f) * m_points[3] +
					20.0f * t * t * (3.0f - 5.0f * t) * m_points[4] +
					20.0f * std::pow(t, 3.0f) * m_points[5];

				float k = std::abs(dp.x * ddp.y - dp.y * ddp.x) / std::pow(dp.length(), 3.0f);

				Vector2 tangent = dp.normal();
				Vector2 normal = tangent.perpendicular();
				Vector2 v = normal * k * scale;

				if (flip)
					v.negate();

				Vector2 curvaturePoint = v + p1;
				result.push_back(curvaturePoint);
			}
			return result;

		}

		float curvature(float t)const
		{
			Vector2 dp = -5.0f * std::pow(1.0f - t, 4.0f) * m_points[0] +
				5.0f * ( 5.0f * t - 1.0f ) * std::pow(t - 1.0f, 3.0f) * m_points[1] +
				-10.0f * (5.0f * t -  2.0f)* (t - 1.0f) * (t - 1.0f) * m_points[2] +
				10.0f * t * t * (5.0f * t * t - 8.0f * t + 3.0f) * m_points[3] +
				5.0f * std::pow(t, 3.0f) * (4.0f - 5.0f * t) * m_points[4] +
				5.0f * std::pow(t, 4.0f) * m_points[5];

			Vector2 ddp = -20.0f * std::pow(t - 1.0f, 3.0f) * m_points[0] +
				20.0f * (5.0f * t - 2.0f) * std::pow(t - 1.0f, 2.0f) * m_points[1] +
				-20.0f * (10.0f * std::pow(t, 3.0f) - 18.0f * t * t + 9.0f * t - 1.0f) * m_points[2] +
				20.0f * (10.0f * t * t - 12.0f * t + 3.0f) * (t - 1.0f) * m_points[3] +
				20.0f * t * t * (3.0f - 5.0f * t) * m_points[4] +
				20.0f * std::pow(t, 3.0f) * m_points[5];

			float k = std::abs(dp.x * ddp.y - dp.y * ddp.x) / std::pow(dp.length(), 3.0f);

			return k;
		}

	private:

		std::array<Vector2, 6> m_points;
	};

	class CubicBezier
	{
	public:

		static CubicBezier fromControlPoints(const Vector2& p0, const Vector2& p1, const Vector2& p2, const Vector2& p3)
		{
			CubicBezier bezier;
			bezier.setPoints(p0, p1, p2, p3);
			return bezier;
		}

		static CubicBezier fromHermite(const Vector2& p0, const Vector2& dir0, const Vector2& p1, const Vector2& dir1)
		{
			CubicBezier bezier;
			bezier.setPoints(p0, p0 + dir0 / 3.0f, p1 - dir1 / 3.0f, p1);
			return bezier;
		}

		float torsion(float t)const
		{
			Vector2 dp = -3.0f * (1.0f - t) * (1.0f - t) * m_points[0] +
				(9.0f * t * t - 12.0f * t + 3.0f) * m_points[1] +
				(6.0f * t - 9.0f * t * t) * m_points[2] +
				3.0f * t * t * m_points[3];

			Vector2 ddp = 6.0f * (1.0f - t) * m_points[0] +
				(18.0f * t - 12.0f) * m_points[1] +
				(6.0f - 18.0f * t) * m_points[2] +
				6.0f * t * m_points[3];

			Vector2 dddp = -6.0f * m_points[0] +
				18.0f * m_points[1] -
				18.0f * m_points[2] +
				6.0f * m_points[3];

			return 0;
		}

		Vector2 sample(float t)const
		{
			return Math::bernstein(t, 0, 3) * m_points[0] +
				Math::bernstein(t, 1, 3) * m_points[1] +
				Math::bernstein(t, 2, 3) * m_points[2] +
				Math::bernstein(t, 3, 3) * m_points[3];
		}

		float curvature(float t)const
		{
			//Vector2 dp = Math::dBernstein(t, 0, 3) * m_points[0] +
			//	Math::dBernstein(t, 1, 3) * m_points[1] +
			//	Math::dBernstein(t, 2, 3) * m_points[2] +
			//	Math::dBernstein(t, 3, 3) * m_points[3];

			Vector2 dp = -3.0f * (1.0f - t) * (1.0f - t) * m_points[0] +
				(9.0f * t * t - 12.0f * t + 3.0f) * m_points[1] +
				(6.0f * t - 9.0f * t * t) * m_points[2] +
				3.0f * t * t * m_points[3];

			Vector2 ddp = 6.0f * (1.0f - t) * m_points[0] +
				(18.0f * t - 12.0f) * m_points[1] +
				(6.0f - 18.0f * t) * m_points[2] +
				6.0f * t * m_points[3];

			float k = std::abs(dp.x * ddp.y - dp.y * ddp.x) / std::pow(dp.length(), 3.0f);

			return k;
		}

		std::vector<Vector2> curvaturePoints(size_t count, float scale = 1.0, bool flip = false) const
		{
			std::vector<Vector2> result;
			result.reserve(count);

			float step = 1.0f / static_cast<float>(count);
			for (float t = 0.0f; t <= 1.0f; t += step)
			{

				Vector2 p1 = sample(t);

				//Vector2 dp = -3.0f * (1.0f - t) * (1.0f - t) * m_points[0] +
				//	(9.0f * t * t - 12.0f * t + 3.0f) * m_points[1] +
				//	(6.0f * t - 9.0f * t * t) * m_points[2] +
				//	3.0f * t * t * m_points[3];

				Vector2 dp = Math::dBernstein(t, 0, 3) * m_points[0] +
					Math::dBernstein(t, 1, 3) * m_points[1] +
					Math::dBernstein(t, 2, 3) * m_points[2] +
					Math::dBernstein(t, 3, 3) * m_points[3];

				Vector2 ddp = Math::d2Bernstein(t, 0, 3) * m_points[0] +
					Math::d2Bernstein(t, 1, 3) * m_points[1] +
					Math::d2Bernstein(t, 2, 3) * m_points[2] +
					Math::d2Bernstein(t, 3, 3) * m_points[3];

				//Vector2 ddp = 6.0f * (1.0f - t) * m_points[0] +
				//	(18.0f * t - 12.0f) * m_points[1] +
				//	(6.0f - 18.0f * t) * m_points[2] +
				//	6.0f * t * m_points[3];

				float k = std::abs(dp.x * ddp.y - dp.y * ddp.x) / std::pow(dp.length(), 3.0f);

				Vector2 tangent = dp.normal();
				Vector2 normal = tangent.perpendicular();
				Vector2 v = normal * k * scale;

				if (flip)
					v.negate();

				Vector2 curvaturePoint = v + p1;
				result.push_back(curvaturePoint);
			}
			return result;

		}

		Vector2& operator[](size_t index)
		{
			return m_points[index];
		}

		Vector2& at(size_t index)
		{
			return m_points[index];
		}

		void setPoints(const Vector2& p0, const Vector2& p1, const Vector2& p2, const Vector2& p3)
		{
			m_points[0] = p0;
			m_points[1] = p1;
			m_points[2] = p2;
			m_points[3] = p3;
		}

		std::vector<Vector2> samplePoints(size_t count)
		{
			std::vector<Vector2> result;
			result.reserve(count);
			float step = 1.0f / static_cast<float>(count);

			for (float t = 0.0f; t <= 1.0f; t += step) 
				result.push_back(sample(t));

			return result;
		}

	private:
		std::array<Vector2, 4> m_points;
	};
	class AbstractSpline
	{
	public:
		virtual ~AbstractSpline() = default;
		virtual std::vector<Vector2> sample(size_t count) = 0;

		void addPoints(const std::vector<Vector2>& points)
		{
			m_points.insert(m_points.end(), points.begin(), points.end());
		}

		Vector2& at(size_t index)
		{
			return m_points[index];
		}

	protected:
		std::vector<Vector2> m_points;
	};

	class CatmullRomSpline : public AbstractSpline
	{

	public:
		std::vector<Vector2> sample(size_t count) override
		{
			std::vector<Vector2> result;

			return result;
		}
		
	};

	class BSpline : public AbstractSpline
	{
	public:
		std::vector<Vector2> sample(size_t count) override
		{
			std::vector<Vector2> result;

			return result;

		}

	private:
	};



	class SolverFrame : public Frame
	{
	public:
		SolverFrame(const FrameSettings& settings) : Frame(settings)
		{
		}



		void onLoad() override
		{
			float min = std::min(m_halfWidth, m_halfHeight);
			float maxRadius = min * m_percentage;
			float radius = maxRadius * m_percentage;
			
			//m_bezierPoints1[1] = Vector2(m_halfWidth - radius, m_halfHeight);
			//m_bezierPoints1[2] = Vector2(m_halfWidth - radius * 0.6f, m_halfHeight);
		}


		void onPostRender(sf::RenderWindow& window) override
		{
			std::vector<Vector2> vertices;
			std::vector<Vector2> newVertices;
			std::vector<Vector2> g3Vertices;

			Vector2 p0(m_halfWidth, 0);
			Vector2 p1(0, m_halfHeight);

			vertices.push_back(p0);
			newVertices.push_back(p1);
			g3Vertices.push_back(p1);

			float min = std::min(m_halfWidth, m_halfHeight);
			float maxRadius = min * m_percentage;
			float radius = maxRadius * m_percentage;

			float step = Constant::Pi * 0.5f / static_cast<float>(m_count);

			Vector2 center(m_halfWidth - radius, m_halfHeight - radius);

			Vector2 corner(radius * std::cos(Math::degreeToRadian(45)), radius * std::sin(Math::degreeToRadian(45)));
			corner += center;

			m_cornerCenter = center;

			for (int i = 0;i <= m_count; ++i)
			{
				float angle = static_cast<float>(i) * step;
				Vector2 p(radius * std::cos(angle), radius * std::sin(angle));
				p += center;

				vertices.push_back(p);
			}

			vertices.push_back(p1);


			//reference line

			if (m_showReferenceLine)
				RenderSFMLImpl::renderLine(window, *m_settings.camera, center, corner, gray);

			Vector2 refP1(m_innerWidthFactor * (m_halfWidth - radius), m_halfHeight);
			Vector2 refP2(m_innerWidthFactor * (m_halfWidth - radius), 0.0f);
			Vector2 refP3(m_halfWidth, m_innerHeightFactor * (m_halfHeight - radius));
			Vector2 refP4(0.0f, m_innerHeightFactor * (m_halfHeight - radius));

			if (m_showReferenceLine)
			{
				RenderSFMLImpl::renderLine(window, *m_settings.camera, refP1, refP2, gray);
				RenderSFMLImpl::renderLine(window, *m_settings.camera, refP3, refP4, gray);
			}

			refP1 = Vector2(m_halfWidth, m_halfHeight - radius);
			refP2 = Vector2(0.0f, m_halfHeight - radius);
			refP3 = Vector2(m_halfWidth - radius, m_halfHeight);
			refP4 = Vector2(m_halfWidth - radius, 0.0f);
			if (m_showReferenceLine)
			{
				RenderSFMLImpl::renderLine(window, *m_settings.camera, refP1, refP2, gray);
				RenderSFMLImpl::renderLine(window, *m_settings.camera, refP3, refP4, gray);
			}
			refP1 = Vector2(corner.x, 0.0f);
			refP2 = Vector2(0.0f, corner.y);
			if (m_showReferenceLine)
			{
				RenderSFMLImpl::renderLine(window, *m_settings.camera, corner, refP1, gray);
				RenderSFMLImpl::renderLine(window, *m_settings.camera, corner, refP2, gray);
			}
		
			float roundedAngle = Math::degreeToRadian(45.0f * m_cornerPercentage);
			float startRounded = Math::degreeToRadian(45.0f) - roundedAngle;
			float endRounded = Math::degreeToRadian(45.0f) + roundedAngle;

			refP1 = radius * Vector2(std::cos(startRounded), std::sin(startRounded));
			refP2 = radius * Vector2(std::cos(endRounded), std::sin(endRounded));
			refP1 += center;
			refP2 += center;

			if (m_showReferenceLine)
			{
				RenderSFMLImpl::renderLine(window, *m_settings.camera, center, refP1, gray);
				RenderSFMLImpl::renderLine(window, *m_settings.camera, center, refP2, gray);
			}

			m_endRoundedPos = refP2;
			m_startRoundedPos = refP1;

			m_bezier1[0]= Vector2(m_innerWidthFactor * (m_halfWidth - radius), m_halfHeight);
			m_bezier1[3] = refP2;

			m_bezier2[0] = Vector2(m_halfWidth, m_innerHeightFactor * (m_halfHeight - radius));
			m_bezier2[3] = refP1;

			Vector2 endDir = m_cornerCenter - m_endRoundedPos;
			endDir = endDir.normal().perpendicular();

			float t = (m_halfHeight - m_endRoundedPos.y) / endDir.y;

			Vector2 p = m_endRoundedPos + t * endDir;

			m_bezier1[2] = p;

			endDir = m_cornerCenter - m_startRoundedPos;
			endDir = endDir.normal().perpendicular();

			t = (m_halfWidth - m_startRoundedPos.x) / endDir.x;

			p = m_startRoundedPos + t * endDir;

			m_bezier2[2] = p;

			float A = 18.0f * ((m_bezier1[3].x - m_bezier1[2].x) * (m_bezier1[0].y - m_bezier1[2].y)
				- (m_bezier1[0].x - m_bezier1[2].x) * (m_bezier1[3].y - m_bezier1[2].y));
			float C = std::pow(9.0f * (m_bezier1[3] - m_bezier1[2]).dot(m_bezier1[3] - m_bezier1[2]), 3.0f);
			C /= radius * radius;

			float u = std::sqrt(C) / std::abs(A);
			Vector2 finalP = m_bezier1[0] * u + (1.0f - u) * m_bezier1[2];

			m_bezier1[1] = finalP;

			A = 18.0f * ((m_bezier2[3].x - m_bezier2[2].x) * (m_bezier2[0].y - m_bezier2[2].y)
				- (m_bezier2[0].x - m_bezier2[2].x) * (m_bezier2[3].y - m_bezier2[2].y));
			C = std::pow(9.0f * (m_bezier2[3] - m_bezier2[2]).dot(m_bezier2[3] - m_bezier2[2]), 3.0f);
			C /= radius * radius;

			u = std::sqrt(C) / std::abs(A);

			finalP = m_bezier2[0] * u + (1.0f - u) * m_bezier2[2];

			m_bezier2[1] = finalP;

			m_quinticBezier[0] = m_bezier1[0];
			m_quinticBezier[5] = m_bezier1[3];

			if (!once)
			{
				m_quinticBezier[1] = Vector2(0.538808107f, m_halfHeight);
				m_quinticBezier[2] = Vector2(0.597935021f, m_halfHeight);
				m_quinticBezier[3] = Vector2(0.672799885f, 0.981727779f);
				m_quinticBezier[4] = Vector2(0.740983009f, 0.947854280f);
				once = true;
			}

			//RenderSFMLImpl::renderPoint(window, *m_settings.camera, m_bezierPoints[0], color, 4.0f);
			//RenderSFMLImpl::renderPoint(window, *m_settings.camera, m_bezierPoints[3], color, 4.0f);
			//RenderSFMLImpl::renderPoint(window, *m_settings.camera, m_bezierPoints[1], gray, 4.0f);
			//RenderSFMLImpl::renderPoint(window, *m_settings.camera, m_bezierPoints[2], gray, 4.0f);

			std::vector<Vector2> bezierPoints1 = m_bezier1.samplePoints(m_bezierCount);
			std::vector<Vector2> bezierPoints2 = m_bezier2.samplePoints(m_bezierCount);

			for(auto iter = bezierPoints1.begin(); iter != bezierPoints1.end() - 1; ++iter)
			{
				newVertices.push_back(*iter);
			}

			float k = 1.0f / radius;
			float scaleK = 1.0f * k;
			//draw curvature of rounded
			std::vector<Vector2> curvatureOfRounded;
			std::vector<Vector2> curvatureOfRoundedStart;
			float roundedStep = endRounded - startRounded;
			float roundedCount = m_count * 0.5f;
			roundedStep /= roundedCount;
			for (float i = 0; i <= roundedCount; i += 1.0f)
			{
				float angle = i * roundedStep + startRounded;
				Vector2 from(std::cos(angle), std::sin(angle));
				from *= radius;
				from += center;
				Vector2 point(std::cos(angle), std::sin(angle));
				point *= scaleK * m_curvatureScaleFactor;
				point += from;

				curvatureOfRounded.push_back(point);
				curvatureOfRoundedStart.push_back(from);

				if(m_showRoundedCurvature)
					RenderSFMLImpl::renderLine(window, *m_settings.camera, from, point, gray);

				if (i == 0)
					continue;

				if (m_showRoundedCurvature)
				{

					RenderSFMLImpl::renderLine(window, *m_settings.camera,
						curvatureOfRounded[curvatureOfRounded.size() - 2], curvatureOfRounded[curvatureOfRounded.size() - 1], gray);

				}
			}

			for(auto iter = curvatureOfRoundedStart.rbegin() + 1; iter != curvatureOfRoundedStart.rend(); ++iter)
			{
				newVertices.push_back(*iter);
			}

			

			if (m_showBezierCurvature)
			{

				std::vector<Vector2> bezierCurvature1 = m_bezier1.curvaturePoints(m_bezierCount, m_curvatureScaleFactor);
				std::vector<Vector2> bezierCurvature2 = m_bezier2.curvaturePoints(m_bezierCount, m_curvatureScaleFactor, true);

				for(size_t i = 0; i < bezierCurvature1.size(); ++i)
				{
					RenderSFMLImpl::renderLine(window, *m_settings.camera, bezierPoints1[i], 
						bezierCurvature1[i], RenderConstant::Green);
					RenderSFMLImpl::renderLine(window, *m_settings.camera, bezierPoints2[i], bezierCurvature2[i], 
						RenderConstant::Green);

					if(i == 0)
						continue;

					//RenderSFMLImpl::renderArrow(window, *m_settings.camera, bezierCurvature1[i], 
					//	bezierCurvature1[i - 1], RenderConstant::Green, 0.3f, 30);

					//RenderSFMLImpl::renderArrow(window, *m_settings.camera, bezierCurvature2[i - 1],
					//	bezierCurvature2[i], RenderConstant::Green, 0.3f, 30);

					RenderSFMLImpl::renderLine(window, *m_settings.camera, bezierCurvature1[i - 1], bezierCurvature1[i], RenderConstant::Green);
					RenderSFMLImpl::renderLine(window, *m_settings.camera, bezierCurvature2[i - 1], bezierCurvature2[i], RenderConstant::Green);
				}

			}

			if (m_showQuinticBezier)
			{
				for (int i = 1; i <= 4; ++i)
				{
					RenderSFMLImpl::renderPoint(window, *m_settings.camera, m_quinticBezier[i], RenderConstant::Gray, 4.0f);
				}

				auto quinticBezierPoints = m_quinticBezier.samplePoints(m_bezierCount);
				auto quinticBezierCurvaturePoints = m_quinticBezier.curvaturePoints(m_bezierCount, m_curvatureScaleFactor);

				for (size_t i = 1; i < quinticBezierPoints.size(); ++i)
				{

					RenderSFMLImpl::renderLine(window, *m_settings.camera, quinticBezierPoints[i - 1], quinticBezierPoints[i], RenderConstant::Yellow);

					if (m_showQuinticBezierCurvature)
						{
						RenderSFMLImpl::renderLine(window, *m_settings.camera, quinticBezierCurvaturePoints[i - 1]
							, quinticBezierCurvaturePoints[i], RenderConstant::Yellow);

						RenderSFMLImpl::renderLine(window, *m_settings.camera, quinticBezierPoints[i], quinticBezierCurvaturePoints[i], RenderConstant::Yellow);

					}
				}
			}

			if(m_showG3)
			{
				Vector2 start = Vector2((m_halfWidth - radius) * m_innerWidthFactor, m_halfHeight);
				Vector2 end = Vector2(m_halfWidth, (m_halfHeight - radius) * m_innerHeightFactor);

				//RenderSFMLImpl::renderLine(window, *m_settings.camera, start, end, RenderConstant::Red);

				m_rationalCubicBezier.pointAt(0) = start;
				m_rationalCubicBezier.pointAt(1) = m_bezier1[1];
				m_rationalCubicBezier.pointAt(2) = m_bezier1[2];
				m_rationalCubicBezier.pointAt(3) = m_endRoundedPos;

				m_rationalCubicBezier2.pointAt(0) = end;
				m_rationalCubicBezier2.pointAt(1) = m_bezier2[1];
				m_rationalCubicBezier2.pointAt(2) = m_bezier2[2];
				m_rationalCubicBezier2.pointAt(3) = m_startRoundedPos;

				if(m_numOptimize)
				{
					float resolution = m_residualResolution;
					{
						//optimize w_0 to make curvature variance at P_3 close to zero
						float k0 = m_rationalCubicBezier.curvature(0.0f);
						float k1 = m_rationalCubicBezier.curvature(1.0f);


						float residual1 = (k0 - m_rationalCubicBezier.curvature(resolution)) / resolution;
						float residual2 = (k1 - m_rationalCubicBezier.curvature(1.0f - resolution)) / resolution;

						//while(std::abs(residual2) > 1e-3)
						//{
						if (residual2 > 0)
							//m_rationalCubicBezier.weightAt(0) += 0.0005f * m_rationalCubicBezier.weightAt(0);
							m_rationalCubicBezier.weightAt(0) *= (1.0f + m_optimizeSpeed);
						else
							//m_rationalCubicBezier.weightAt(0) -= 0.0005f * m_rationalCubicBezier.weightAt(0);
							m_rationalCubicBezier.weightAt(0) *= (1.0f - m_optimizeSpeed);

						//	residual2 = (k1 - m_rationalCubicBezier.curvature(0.9999f)) / 0.0001f;
						//	std::cout << "optimize res2:" << residual2 << "\n";
						//}

						float residual3 = k1 - scaleK;

						if (residual3 > 0)
							m_rationalCubicBezier.weightAt(1) *= 0.9999f;
						else
							m_rationalCubicBezier.weightAt(1) *= 1.0001f;

						std::cout << "res1:" << residual1 << ", res2:" << residual2 << ", res3:" << residual3 << ", w0:" << m_rationalCubicBezier.weightAt(0) << "\n";

					}

					{
						//optimize w_0 to make curvature variance at P_3 close to zero
						float k0 = m_rationalCubicBezier2.curvature(0.0f);
						float k1 = m_rationalCubicBezier2.curvature(1.0f);

						float residual1 = (k0 - m_rationalCubicBezier.curvature(resolution)) / resolution;
						float residual2 = (k1 - m_rationalCubicBezier.curvature(1.0f - resolution)) / resolution;

						//while(std::abs(residual2) > 1e-3)
						//{
						if (residual2 > 0)
							//m_rationalCubicBezier2.weightAt(0) += 0.0005f * m_rationalCubicBezier2.weightAt(0);
							m_rationalCubicBezier2.weightAt(0) *= (1.0f + m_optimizeSpeed);
						else
							//m_rationalCubicBezier2.weightAt(0) -= 0.0005f * m_rationalCubicBezier2.weightAt(0);
							m_rationalCubicBezier2.weightAt(0) *= (1.0f - m_optimizeSpeed);

						//	residual2 = (k1 - m_rationalCubicBezier2.curvature(0.9999f)) / 0.0001f;
						//	std::cout << "optimize res2:" << residual2 << "\n";
						//}

						float residual3 = k1 - scaleK;

						if (residual3 > 0)
							m_rationalCubicBezier2.weightAt(1) *= 0.9999f;
						else
							m_rationalCubicBezier2.weightAt(1) *= 1.0001f;

						//std::cout << "res1:" << residual1 << ", res2:" << residual2 << ", res3:" << residual3 << ", w0:" << m_rationalCubicBezier2.weightAt(0) << "\n";
					}
				}
				

				

				{
					std::vector<Vector2> rationalCubicBezierPoints = m_rationalCubicBezier.samplePoints(m_bezierCount);
					std::vector<Vector2> rationalCubicBezierCurvaturePoints = m_rationalCubicBezier.curvaturePoints(m_bezierCount, m_curvatureScaleFactor);

					for (auto iter = rationalCubicBezierPoints.begin(); iter != rationalCubicBezierPoints.end() - 1; ++iter)
						g3Vertices.push_back(*iter);

					

					if(m_showG3Curvature)
					{
						for (size_t i = 1; i < rationalCubicBezierPoints.size(); ++i)
						{

							//RenderSFMLImpl::renderLine(window, *m_settings.camera, rationalCubicBezierPoints[i - 1], rationalCubicBezierPoints[i], RenderConstant::Red);


							RenderSFMLImpl::renderLine(window, *m_settings.camera, rationalCubicBezierCurvaturePoints[i - 1]
								, rationalCubicBezierCurvaturePoints[i], RenderConstant::Blue);

							RenderSFMLImpl::renderLine(window, *m_settings.camera, rationalCubicBezierPoints[i], rationalCubicBezierCurvaturePoints[i], RenderConstant::Blue);


						}

					}
					
				}

				for (auto iter = curvatureOfRoundedStart.rbegin();
					iter != curvatureOfRoundedStart.rend(); ++iter)
				{
					g3Vertices.push_back(*iter);
				}

				{
					std::vector<Vector2> rationalCubicBezierPoints = m_rationalCubicBezier2.samplePoints(m_bezierCount);

					for (auto iter = rationalCubicBezierPoints.rbegin() + 1; iter != rationalCubicBezierPoints.rend(); ++iter)
						g3Vertices.push_back(*iter);


					if(m_showG3Curvature)
					{
						std::vector<Vector2> rationalCubicBezierCurvaturePoints = m_rationalCubicBezier2.curvaturePoints(m_bezierCount, m_curvatureScaleFactor, true);

						for (size_t i = 1; i < rationalCubicBezierPoints.size(); ++i)
						{

							//RenderSFMLImpl::renderLine(window, *m_settings.camera, rationalCubicBezierPoints[i - 1], rationalCubicBezierPoints[i], RenderConstant::Green);


							RenderSFMLImpl::renderLine(window, *m_settings.camera, rationalCubicBezierCurvaturePoints[i - 1]
								, rationalCubicBezierCurvaturePoints[i], RenderConstant::Blue);

							RenderSFMLImpl::renderLine(window, *m_settings.camera, rationalCubicBezierPoints[i], rationalCubicBezierCurvaturePoints[i], RenderConstant::Blue);


						}
					}
				}

				g3Vertices.push_back(p0);

			}



			//RenderSFMLImpl::renderPoint(window, *m_settings.camera, finalP, color, 4.0f);

			for(auto&& iter = bezierPoints2.rbegin() + 1; iter != bezierPoints2.rend(); ++iter)
			{
				newVertices.push_back(*iter);
			}

			newVertices.push_back(p0);



			std::vector<Vector2> vertices2, vertices3, vertices4;
			std::vector<Vector2> newVertices2, newVertices3, newVertices4;
			std::vector<Vector2> g3Vertices2, g3Vertices3, g3Vertices4;
			for(auto& p : vertices)
			{
				vertices2.emplace_back(-p.x, p.y);
				vertices3.emplace_back(-p.x, -p.y);
				vertices4.emplace_back(p.x, -p.y);
			}

			for(auto& p : newVertices)
			{
				newVertices2.emplace_back(-p.x, p.y);
				newVertices3.emplace_back(-p.x, -p.y);
				newVertices4.emplace_back(p.x, -p.y);
			}

			for (auto& p : g3Vertices)
			{
				g3Vertices2.emplace_back(-p.x, p.y);
				g3Vertices3.emplace_back(-p.x, -p.y);
				g3Vertices4.emplace_back(p.x, -p.y);
			}
			if (m_showG1)
			{
				for (size_t i = 0; i < vertices.size() - 1; ++i)
				{
					RenderSFMLImpl::renderLine(window, *m_settings.camera, vertices[i], vertices[(i + 1) % vertices.size()], RenderConstant::Red);
					RenderSFMLImpl::renderLine(window, *m_settings.camera, vertices2[i], vertices2[(i + 1) % vertices2.size()], RenderConstant::Red);
					RenderSFMLImpl::renderLine(window, *m_settings.camera, vertices3[i], vertices3[(i + 1) % vertices3.size()], RenderConstant::Red);
					RenderSFMLImpl::renderLine(window, *m_settings.camera, vertices4[i], vertices4[(i + 1) % vertices4.size()], RenderConstant::Red);
				}
			}
			if (m_showG2)
			{
				//for(size_t i = 0 ; i < newVertices.size(); ++i)
				//{
				//	RenderSFMLImpl::renderUInt(window, *m_settings.camera, newVertices[i], *m_settings.font,
				//		i, RenderConstant::Green, 18, Vector2(0.0f, 0.0f));
				//	//RenderSFMLImpl::renderPoint(window, *m_settings.camera, elem, RenderConstant::Green, 2.0f);
				//}
				for (size_t i = 1; i < newVertices.size(); ++i)
				{
					RenderSFMLImpl::renderLine(window, *m_settings.camera, newVertices[i - 1], newVertices[i], RenderConstant::Green);
					RenderSFMLImpl::renderLine(window, *m_settings.camera, newVertices2[i - 1], newVertices2[i], RenderConstant::Green);
					RenderSFMLImpl::renderLine(window, *m_settings.camera, newVertices3[i - 1], newVertices3[i], RenderConstant::Green);
					RenderSFMLImpl::renderLine(window, *m_settings.camera, newVertices4[i - 1], newVertices4[i], RenderConstant::Green);
				}
			}
			if(m_showG3)
			{
				//for(size_t i = 0 ; i < g3Vertices.size(); ++i)
				//{
				//	RenderSFMLImpl::renderUInt(window, *m_settings.camera, g3Vertices[i], *m_settings.font,
				//		i, RenderConstant::Green, 18, Vector2(0.0f, 0.0f));
				//	//RenderSFMLImpl::renderPoint(window, *m_settings.camera, elem, RenderConstant::Green, 2.0f);
				//}
				for (size_t i = 1; i < g3Vertices.size(); ++i)
				{
					RenderSFMLImpl::renderLine(window, *m_settings.camera, g3Vertices[i - 1], g3Vertices[i], RenderConstant::Blue);
					RenderSFMLImpl::renderLine(window, *m_settings.camera, g3Vertices2[i - 1], g3Vertices2[i], RenderConstant::Blue);
					RenderSFMLImpl::renderLine(window, *m_settings.camera, g3Vertices3[i - 1], g3Vertices3[i], RenderConstant::Blue);
					RenderSFMLImpl::renderLine(window, *m_settings.camera, g3Vertices4[i - 1], g3Vertices4[i], RenderConstant::Blue);
				}
			}
			
		}

		void onPostStep(real dt) override
		{
			
		}

		void onKeyPressed(sf::Event& event) override
		{

		}
		void onRenderUI() override
		{

			ImGui::Begin("Rounded Rect");

			ImGui::DragFloat("Half Width", &m_halfWidth, 0.01f, 0.5f, 50.0f);
			ImGui::DragFloat("Half Height", &m_halfHeight, 0.01f, 0.5f, 50.0f);
			ImGui::DragFloat("Rounded Radius Percentage", &m_percentage, 0.005f, 0.005f, 1.0f);
			ImGui::DragFloat("Inner Width Percentage", &m_innerWidthFactor, 0.001f, 0.0f, 1.0f);
			ImGui::DragFloat("Inner Height Percentage", &m_innerHeightFactor, 0.001f, 0.0f, 1.0f);
			ImGui::DragFloat("Corner Angle Percentage", &m_cornerPercentage, 0.005f, 0.0f, 0.995f);
			ImGui::DragInt("Bezier Sample Count", &m_bezierCount, 1, 8, 500);
			ImGui::DragInt("Circle Segment Count", &m_count, 2, 4, 500);
			ImGui::DragFloat("Curvature Scale Factor", &m_curvatureScaleFactor, 0.01f, 0.01f, 1.0f);

			ImGui::DragFloat("Bezier1 Weight P0", &m_rationalCubicBezier.weightAt(0), 1e-6f, 1e-7f, 5.0f, "%.7f");
			ImGui::DragFloat("Bezier1 Weight P1", &m_rationalCubicBezier.weightAt(1), 1e-6f, 1e-7f, 5.0f, "%.7f");
			ImGui::DragFloat("Bezier1 Weight P2", &m_rationalCubicBezier.weightAt(2), 1e-6f, 1e-7f, 5.0f, "%.7f");
			ImGui::DragFloat("Bezier1 Weight P3", &m_rationalCubicBezier.weightAt(3), 1e-6f, 1e-7f, 5.0f, "%.7f");


			ImGui::DragFloat("Bezier2 Weight P0", &m_rationalCubicBezier2.weightAt(0), 1e-6f, 1e-7f, 5.0f, "%.7f");
			ImGui::DragFloat("Bezier2 Weight P1", &m_rationalCubicBezier2.weightAt(1), 1e-6f, 1e-7f, 5.0f, "%.7f");
			ImGui::DragFloat("Bezier2 Weight P2", &m_rationalCubicBezier2.weightAt(2), 1e-6f, 1e-7f, 5.0f, "%.7f");
			ImGui::DragFloat("Bezier2 Weight P3", &m_rationalCubicBezier2.weightAt(3), 1e-6f, 1e-7f, 5.0f, "%.7f");

			ImGui::DragFloat("Optimize Residual Resolution", &m_residualResolution, 1e-6f, 1e-6f, 0.1f, "%.7f");
			ImGui::DragFloat("Optimize Speed", &m_optimizeSpeed, 1e-6f, 1e-7f, 0.1f, "%.7f");
			ImGui::Checkbox("Optimize", &m_numOptimize);

			ImGui::Checkbox("Show Rounded Curvature", &m_showRoundedCurvature);
			ImGui::Checkbox("Show Bezier Curvature", &m_showBezierCurvature);
			ImGui::Checkbox("Show Reference Line", &m_showReferenceLine);
			ImGui::Checkbox("Show Quintic Bezier Curve", &m_showQuinticBezier);
			ImGui::Checkbox("Show Quintic Bezier Curvature", &m_showQuinticBezierCurvature);
			ImGui::Checkbox("Show G1 Continuity", &m_showG1);
			ImGui::Checkbox("Show G2 Continuity", &m_showG2);
			ImGui::Checkbox("Show G3 Continuity", &m_showG3);
			ImGui::Checkbox("Show G3 Curvature", &m_showG3Curvature);

			ImGui::End();
		}
		void onUnLoad() override
		{

		}

		void onMousePress(sf::Event& event) override
		{
			if (event.mouseButton.button == sf::Mouse::Left)
			{
				Vector2 pos(static_cast<real>(event.mouseButton.x), static_cast<real>(event.mouseButton.y));
				m_mousePos = m_settings.camera->screenToWorld(pos);

				for(int i = 1; i <= 4; ++i)
				{
					if ((m_mousePos - m_quinticBezier[i]).length() < 0.01f)
					{
						m_isMoving = true;
						m_currentIndex = i;
						break;
					}
				}

			}
		}
		void onMouseRelease(sf::Event& event) override
		{
			m_currentIndex = -1;
			m_isMoving = false;
		}
		void onMouseMove(sf::Event& event) override
		{
			Vector2 pos(static_cast<real>(event.mouseMove.x), static_cast<real>(event.mouseMove.y));
			m_mousePos = m_settings.camera->screenToWorld(pos);
			if (m_isMoving && m_currentIndex != -1)
			{
				if(m_currentIndex == 1)
				{
					m_quinticBezier[m_currentIndex].x = m_mousePos.x;
				}
				else if(m_currentIndex == 2)
				{
					m_quinticBezier[m_currentIndex].x = m_mousePos.x;
				}
				else if(m_currentIndex == 4)
				{
					if(!m_cornerCenter.isOrigin() && !m_endRoundedPos.isOrigin())
					{
						Vector2 endDir = m_cornerCenter - m_endRoundedPos;
						endDir = endDir.normal().perpendicular();

						float dot = endDir.dot(m_mousePos - m_endRoundedPos);

						Vector2 p = m_endRoundedPos + dot * endDir;
						m_quinticBezier[m_currentIndex] = p;
					}
				}
				else
					m_quinticBezier[m_currentIndex] = m_mousePos;
			}
			//std::cout << "mouse:(" << m_mousePos.x << "," << m_mousePos.y << ")\n";
			//std::cout << "bezier[1]:(" << m_bezierPoints[1].x << "," << m_bezierPoints[1].y << ")\n";
			//std::cout << "bezier[2]:(" << m_bezierPoints[2].x << "," << m_bezierPoints[2].y << ")\n";
		}

	private:
		bool m_numOptimize = false;
		bool once = false;
		bool m_showG1 = false;
		bool m_showG2 = true;
		bool m_showG3 = true;
		bool m_showG3Curvature = true;
		bool m_showRoundedCurvature = true;
		bool m_showBezierCurvature = false;
		bool m_showQuinticBezier = false;
		bool m_showQuinticBezierCurvature = false;
		bool m_showReferenceLine = true;

		int m_currentIndex = -1;
		bool m_isMoving = false;
		Vector2 m_mousePos;
		Vector2 m_cornerCenter;

		Vector2 m_startRoundedPos;
		Vector2 m_endRoundedPos;

		RationalCubicBezier m_rationalCubicBezier2;
		RationalCubicBezier m_rationalCubicBezier;

		QuinticBezier m_quinticBezier;

		CubicBezier m_bezier1;
		CubicBezier m_bezier2;

		//std::array<Vector2, 4> m_bezierPoints1;
		//std::array<Vector2, 4> m_bezierPoints2;

		float m_residualResolution = 1e-5f;
		float m_optimizeSpeed = 1e-5f;
		float m_innerWidthFactor = 0.7f;
		float m_innerHeightFactor = 0.7f;
		float m_curvatureScaleFactor = 0.61f;
		int m_bezierCount = 50;
		int m_count = 50;
		float m_halfWidth = 2.0f;
		float m_halfHeight = 2.0f;
		float m_percentage = 0.67f;

		float m_cornerPercentage = 0.38f;
		sf::Color color = RenderConstant::Cyan;
		sf::Color gray = sf::Color(158, 158, 158, 255);
		
	};
}
#endif
