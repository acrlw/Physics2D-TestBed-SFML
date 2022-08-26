#include "../../../include/math/linear/mat3.h"

namespace Physics2D
{
	Mat3::Mat3()
	{
		clear();
	}

	Mat3::Mat3(const Mat3& mat)
	{
		set(mat);
	}

	Mat3::Mat3(const Vec3& _r0, const Vec3& _r1, const Vec3& _r2)
		: r0(_r0), r1(_r1), r2(_r2)
	{
	}

	Mat3::Mat3(
		const real& e00, const real& e01, const real& e02,
		const real& e10, const real& e11, const real& e12,
		const real& e20, const real& e21, const real& e22)
	{
		set(
			e00, e01, e02,
			e10, e11, e12,
			e20, e21, e22);
	}

	Vec3 Mat3::operator[](uint8_t index)
	{
		return v[index];
	}

	Mat3& Mat3::operator=(const Mat3& rhs)
	{
		if (&rhs != this)
			set(rhs);

		return *this;
	}

	Mat3& Mat3::operator+=(const Mat3& rhs)
	{

		return *this;
	}

	Mat3& Mat3::operator-=(const Mat3& rhs)
	{

		return *this;
	}

	Mat3& Mat3::operator*=(const real& factor)
	{

		return *this;
	}

	Mat3& Mat3::operator/=(const real& factor)
	{
		assert(!realEqual(factor, 0));

		return *this;
	}

	Vec3 Mat3::c0() const
	{
		return Vec3(r0.x, r1.x, r2.x);
	}

	Vec3 Mat3::c1() const
	{
		return Vec3(r0.y, r1.y, r2.y);
	}

	Vec3 Mat3::c2() const
	{
		return Vec3(r0.z, r1.z, r2.z);
	}

	real Mat3::e00() const
	{
		return f[0];
	}

	real Mat3::e01() const
	{
		return f[1];
	}

	real Mat3::e02() const
	{
		return f[2];
	}

	real Mat3::e10() const
	{
		return f[4];
	}

	real Mat3::e11() const
	{
		return f[5];
	}

	real Mat3::e12() const
	{
		return f[6];
	}

	real Mat3::e20() const
	{
		return f[8];
	}

	real Mat3::e21() const
	{
		return f[9];
	}

	real Mat3::e22() const
	{
		return f[10];
	}


	real Mat3::determinant() const
	{
		return determinant(*this);
	}

	Mat3& Mat3::transpose()
	{
		realSwap(v[0].y, v[1].x);
		realSwap(v[0].z, v[2].x);
		realSwap(v[1].z, v[2].y);
		return *this;
	}

	Mat3& Mat3::invert()
	{
		invert(*this);
		return *this;
	}

	Mat3& Mat3::clear()
	{
		r0.clear();
		r1.clear();
		r2.clear();
		return *this;
	}


	Mat3& Mat3::set(
		const real& e00, const real& e01, const real& e02,
		const real& e10, const real& e11, const real& e12,
		const real& e20, const real& e21, const real& e22)
	{
		r0.set(e00, e01, e02);
		r1.set(e10, e11, e12);
		r2.set(e20, e21, e22);
		return *this;
	}

	Mat3& Mat3::set(const Vec3& _r0, const Vec3& _r1, const Vec3& _r2)
	{
		r0.set(_r0);
		r1.set(_r1);
		r2.set(_r2);
		return *this;
	}

	Mat3& Mat3::set(const Mat3& other)
	{
		r0.set(other.r0);
		r1.set(other.r1);
		r2.set(other.r2);
		return *this;
	}

	Vec3 Mat3::multiply(const Vec3& rhs) const
	{
		return multiply(*this, rhs);
	}

	Mat3& Mat3::multiply(const Mat3& rhs)
	{
		*this = multiply(*this, rhs);
		return *this;
	}

	Mat3 Mat3::skewSymmetricMatrix(const Vec3& v)
	{
		return Mat3(
			0, v.z, -v.y,
			-v.z, 0, v.x,
			v.y, -v.x, 0);
	}

	Mat3 Mat3::identityMatrix()
	{
		return Mat3(
			1, 0, 0,
			0, 1, 0,
			0, 0, 1);
	}

	Mat3 Mat3::multiply(const Mat3& lhs, const Mat3& rhs)
	{
		return Mat3();
	}

	Vec3 Mat3::multiply(const Mat3& lhs, const Vec3& rhs)
	{
		return Vec3();
	}

	real Mat3::determinant(const Mat3& mat)
	{

		return real();
	}

	Mat3 Mat3::rotate(const Vec3& axis, const real& deg)
	{
		return Mat3();
	}

	bool Mat3::invert(Mat3& mat)
	{
		const real det = mat.determinant();
		if (realEqual(det, 0.0f))
			return false;

		return true;
	}
}
