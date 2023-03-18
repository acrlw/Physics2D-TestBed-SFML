#include "physics2d_mat2.h"

namespace Physics2D
{
	
	Mat2::Mat2()
	{
		clear();
	}

	Mat2::Mat2(const real& radian)
	{
		set(radian);
	}


	Mat2::Mat2(const Vec2& _r0, const Vec2& _r1) : r0(_r0), r1(_r1)
	{
	}

	Mat2::Mat2(
		const real& e00, const real& e01,
		const real& e10, const real& e11)
		: data(_mm_setr_ps(e00, e01, e10, e11))
	{
	}

	Mat2::Mat2(__m128 d)
	{
		_mm_store_ps(f, d);
	}

	Mat2::Mat2(const Mat2& mat)
	{
		set(mat);
	}



	Mat2& Mat2::operator=(const Mat2& rhs)
	{
		if (&rhs != this)
			set(rhs);
		return *this;
	}

	Mat2& Mat2::operator+=(const Mat2& rhs)
	{
		_mm_store_ps(f, _mm_add_ps(data, rhs.data));
		return *this;
	}

	Mat2& Mat2::operator-=(const Mat2& rhs)
	{
		_mm_store_ps(f, _mm_sub_ps(data, rhs.data));
		return *this;
	}

	Mat2& Mat2::operator*=(const real& factor)
	{
		_mm_store_ps(f, _mm_mul_ps(data, _mm_set1_ps(factor)));
		return *this;
	}

	Mat2& Mat2::operator/=(const real& factor)
	{
		checkZero(factor);
		_mm_store_ps(f, _mm_div_ps(data, _mm_set1_ps(factor)));
		return *this;
	}

	Mat2 Mat2::operator+(const Mat2& rhs) const
	{
		return Mat2(_mm_add_ps(data, rhs.data));
	}

	Mat2 Mat2::operator-(const Mat2& rhs) const
	{
		return Mat2(_mm_sub_ps(data, rhs.data));
	}

	Vec2& Mat2::operator[](uint8_t index)
	{
		return v[index];
	}

	Vec2 Mat2::c0() const
	{
		return Vec2(f[0], f[2]);
	}

	Vec2 Mat2::c1() const
	{
		return Vec2(f[1], f[3]);
	}

	real Mat2::e00()const
	{
		return f[0];
	}
	real Mat2::e01()const
	{
		return f[1];
	}
	real Mat2::e10()const
	{
		return f[2];
	}
	real Mat2::e11()const
	{
		return f[3];
	}

	real Mat2::determinant() const
	{
		return determinant(*this);
	}

	Mat2& Mat2::transpose()
	{
		realSwap(r1.x, r0.y);
		return *this;
	}

	Mat2& Mat2::invert()
	{
		invert(*this);
		return *this;
	}

	Mat2& Mat2::multiply(const Mat2& rhs)
	{
		*this = multiply(*this, rhs);
		return *this;
	}

	Vec2 Mat2::multiply(const Vec2& rhs) const
	{
		return multiply(*this, rhs);
	}

	Mat2& Mat2::clear()
	{
		_mm_store_ps(f, _mm_setzero_ps());
		return *this;
	}

	Mat2& Mat2::set(const real& c0x, const real& c0y, const real& c1x, const real& c1y)
	{
		_mm_store_ps(f, _mm_setr_ps(c0x, c0y, c1x, c1y));
		return *this;
	}

	Mat2& Mat2::set(const Vec2& _c0, const Vec2& _c1)
	{
		_mm_store_ps(f, _mm_setr_ps(_c0.x, _c0.y, _c1.x, _c1.y));
		return *this;
	}

	Mat2& Mat2::set(const Mat2& other)
	{
		_mm_store_ps(f, other.data);
		return *this;
	}

	Mat2& Mat2::set(const real& radian)
	{
		const real c = cosx(radian);
		const real s = sinx(radian);
		_mm_store_ps(f, _mm_setr_ps(c, -s, s, c));
		return *this;
	}

	Mat2& Mat2::swap(Mat2& other)
	{
		__m128 temp = other.data;
		_mm_store_ps(f, other.data);
		_mm_store_ps(other.f, temp);
		return *this;
	}

	Mat2 Mat2::skewSymmetricMatrix(const Vec2& r)
	{
		return Mat2(0, -r.y, r.x, 0);
	}

	Mat2 Mat2::identity()
	{
		return Mat2(
			1, 0,
			0, 1);
	}

	Vec2 Mat2::multiply(const Mat2& lhs, const Vec2& rhs)
	{
		//rhs is considered as column vector

		__m128 r = _mm_setr_ps(rhs.x, rhs.y, rhs.x, rhs.y);
		r = _mm_mul_ps(lhs.data, r);
		const float* result = reinterpret_cast<float*>(&r);
		return Vec2(result[0] + result[1], result[2] + result[3]);
	}

	Mat2 Mat2::multiply(const Mat2& lhs, const Mat2& rhs)
	{
		//lhs: x0 y0 x0 y0
		//rhs: x0 x0 x1 x1

		//lhs: x1 y1 x1 y1
		//rhs: y0 y0 y1 y1

		__m128 l1 = _mm_shuffle_ps(lhs.data, lhs.data, _MM_SHUFFLE(1, 0, 1, 0));
		__m128 r1 = _mm_shuffle_ps(rhs.data, rhs.data, _MM_SHUFFLE(2, 2, 0, 0));

		__m128 l2 = _mm_shuffle_ps(lhs.data, lhs.data, _MM_SHUFFLE(3, 2, 3, 2));
		__m128 r2 = _mm_shuffle_ps(rhs.data, rhs.data, _MM_SHUFFLE(3, 3, 1, 1));


		return { _mm_add_ps(_mm_mul_ps(l1, r1), _mm_mul_ps(l2, r2)) };
	}

	real Mat2::determinant(const Mat2& mat)
	{
		return mat.r0.x * mat.r1.y - mat.r0.y * mat.r1.x;
	}

	bool Mat2::invert(Mat2& mat)
	{
		const real det = mat.determinant();

		if (realEqual(det, 0))
			return false;

		realSwap(mat.f[0], mat.f[3]);
		mat.f[1] *= -1;
		mat.f[2] *= -1;
		mat /= det;
		return true;
	}
}
