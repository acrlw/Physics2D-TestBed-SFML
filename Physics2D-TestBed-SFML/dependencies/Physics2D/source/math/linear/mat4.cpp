#include "../../../include/math/linear/mat4.h"

namespace Physics2D
{
	Mat4::Mat4()
	{
		clear();
	}

	Mat4::Mat4(const Vec4& r0, const Vec4& r1, const Vec4& r2, const Vec4& r3)
	{
		set(r0, r1, r2, r3);
	}
	Mat4::Mat4(const real& e00, const real& e01, const real& e02, const real& e03,
		const real& e10, const real& e11, const real& e12, const real& e13,
		const real& e20, const real& e21, const real& e22, const real& e23,
		const real& e30, const real& e31, const real& e32, const real& e33)
	{
		set(
			e00, e01, e02, e03,
			e10, e11, e12, e13,
			e20, e21, e22, e23,
			e30, e31, e32, e33);
	}
	Mat4::Mat4(const Mat3& mat)
	{
		set(mat);
	}

	Mat4& Mat4::operator=(const Mat3& rhs)
	{
		set(rhs);
		return *this;
	}

	Mat4& Mat4::operator=(const Mat4& rhs)
	{
		set(rhs);
		return *this;
	}

	Mat4 Mat4::operator+(const Mat4& rhs) const
	{
		Mat4 temp(*this);
		temp += rhs;
		return temp;
	}

	Mat4 Mat4::operator-(const Mat4& rhs) const
	{
		Mat4 temp(*this);
		temp -= rhs;
		return temp;
	}

	Vec4 Mat4::operator[](uint8_t index) const
	{
		return v[index];
	}

	Mat4& Mat4::operator+=(const Mat4& rhs)
	{
		_mm256_store_ps(&f[0], _mm256_add_ps(data[0], rhs.data[0]));
		_mm256_store_ps(&f[8], _mm256_add_ps(data[1], rhs.data[1]));
		return *this;
	}

	Mat4& Mat4::operator-=(const Mat4& rhs)
	{
		_mm256_store_ps(&f[0], _mm256_sub_ps(data[0], rhs.data[0]));
		_mm256_store_ps(&f[8], _mm256_sub_ps(data[1], rhs.data[1]));
		return *this;
	}

	Mat4& Mat4::operator*=(const real& factor)
	{
		_mm256_store_ps(&f[0], _mm256_mul_ps(data[0], _mm256_set1_ps(factor)));
		_mm256_store_ps(&f[8], _mm256_mul_ps(data[1], _mm256_set1_ps(factor)));
		return *this;
	}

	Mat4& Mat4::operator/=(const real& factor)
	{
		checkZero(factor);
		_mm256_store_ps(&f[0], _mm256_div_ps(data[0], _mm256_set1_ps(factor)));
		_mm256_store_ps(&f[8], _mm256_div_ps(data[1], _mm256_set1_ps(factor)));
		return *this;
	}

	Vec4 Mat4::c0() const
	{
		return Vec4(r0.x, r1.x,
			r2.x, r3.x);
	}

	Vec4 Mat4::c1() const
	{
		return Vec4(r0.y, r1.y,
			r2.y, r3.y);
	}

	Vec4 Mat4::c2() const
	{
		return Vec4(r0.z, r1.z,
			r2.z, r3.z);
	}

	Vec4 Mat4::c3() const
	{
		return Vec4(r0.w, r1.w,
			r2.w, r3.w);
	}

	real Mat4::e00() const
	{
		return f[0];
	}

	real Mat4::e01() const
	{
		return f[1];
	}

	real Mat4::e02() const
	{
		return f[2];
	}

	real Mat4::e03() const
	{
		return f[3];
	}

	real Mat4::e10() const
	{
		return f[4];
	}

	real Mat4::e11() const
	{
		return f[5];
	}

	real Mat4::e12() const
	{
		return f[6];
	}

	real Mat4::e13() const
	{
		return f[7];
	}

	real Mat4::e20() const
	{
		return f[8];
	}

	real Mat4::e21() const
	{
		return f[9];
	}

	real Mat4::e22() const
	{
		return f[10];
	}

	real Mat4::e23() const
	{
		return f[11];
	}

	real Mat4::e30() const
	{
		return f[12];
	}
	real Mat4::e31() const
	{
		return f[13];
	}
	real Mat4::e32() const
	{
		return f[14];
	}

	real Mat4::e33() const
	{
		return f[15];
	}

	Mat4& Mat4::set(
		const real& _e00, const real& _e01, const real& _e02, const real& _e03,
		const real& _e10, const real& _e11, const real& _e12, const real& _e13,
		const real& _e20, const real& _e21, const real& _e22, const real& _e23,
		const real& _e30, const real& _e31, const real& _e32, const real& _e33)
	{

		_mm256_store_ps(&f[0],
			_mm256_setr_ps(_e00, _e01, _e02, _e03, _e10, _e11, _e12, _e13));
		_mm256_store_ps(&f[8],
			_mm256_setr_ps(_e20, _e21, _e22, _e23, _e30, _e31, _e32, _e33));
		return *this;
	}

	Mat4& Mat4::set(const Vec4& _r0, const Vec4& _r1, const Vec4& _r2, const Vec4& _r3)
	{
		_mm256_store_ps(&f[0],
			_mm256_setr_m128(_r0.data, _r1.data));
		_mm256_store_ps(&f[8],
			_mm256_setr_m128(_r2.data, _r3.data));
		return *this;
	}

	Mat4& Mat4::set(const Mat4& other)
	{
		_mm256_store_ps(&f[0],
			other.data[0]);
		_mm256_store_ps(&f[8],
			other.data[1]);
		return *this;
	}

	Mat4& Mat4::set(const Mat3& other)
	{
		r0.set(other.r0);
		r1.set(other.r1);
		r2.set(other.r2);
		r3.clear();
		return *this;
	}

	Mat4& Mat4::clear()
	{
		_mm256_store_ps(&f[0],
			_mm256_setzero_ps());
		_mm256_store_ps(&f[8],
			_mm256_setzero_ps());
		return *this;
	}

	Vec4 Mat4::multiply(const Vec4& rhs) const
	{
		return multiply(*this, rhs);
	}

	Mat4& Mat4::multiply(const Mat4& rhs)
	{
		*this = multiply(*this, rhs);
		return *this;
	}

	real Mat4::determinant() const
	{
		return determinant(*this);
	}

	Mat4& Mat4::transpose()
	{
		set(c0(), c1(), c2(), c3());
		return *this;
	}

	Mat4& Mat4::invert()
	{
		invert(*this);
		return *this;
	}

	Mat4 Mat4::identity()
	{
		return {
			1, 0, 0, 0,
			0, 1, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1 };
	}

	Mat4 Mat4::multiply(const Mat4& lhs, const Mat4& rhs)
	{
		Mat4 result{
			multiply(lhs.r0, rhs) ,
			multiply(lhs.r1, rhs) ,
			multiply(lhs.r2, rhs) ,
			multiply(lhs.r3, rhs) };
		return result;
	}

	Vec4 Mat4::multiply(const Mat4& lhs, const Vec4& rhs)
	{
		//lhs: x0 y0 z0 w0
		//rhs: x0 y0 z0 w0
		__m256 r = _mm256_set_m128(rhs.data, rhs.data);
		__m256 temp1 = _mm256_dp_ps(r, lhs.data[0], 0xff);
		__m256 temp2 = _mm256_dp_ps(r, lhs.data[1], 0xff);
		float* r01 = reinterpret_cast<float*>(&temp1);
		float* r02 = reinterpret_cast<float*>(&temp2);
		//lhs:
		return { _mm_setr_ps(r01[0], r01[4], r02[0], r02[4]) };
	}
	Vec4 Mat4::multiply(const Vec4& lhs, const Mat4& rhs)
	{
		//lhs: x0 y0 z0 w0
		//rhs: x0 x1 x2 x3
		__m256 r = _mm256_set_m128(lhs.data, lhs.data);
		Vec4 c0 = rhs.c0();
		Vec4 c1 = rhs.c1();
		Vec4 c2 = rhs.c2();
		Vec4 c3 = rhs.c3();
		__m256 rhsC0C1 = _mm256_setr_m128(c0.data, c1.data);
		__m256 rhsC2C3 = _mm256_setr_m128(c2.data, c3.data);
		__m256 temp1 = _mm256_dp_ps(r, rhsC0C1, 0xff);
		__m256 temp2 = _mm256_dp_ps(r, rhsC2C3, 0xff);
		float* r01 = reinterpret_cast<float*>(&temp1);
		float* r02 = reinterpret_cast<float*>(&temp2);
		return { _mm_setr_ps(r01[0], r01[4], r02[0], r02[4]) };
	}
	real Mat4::determinant(const Mat4& mat)
	{
		return real();
	}

	bool Mat4::invert(Mat4& mat)
	{


		return true;
	}
}
