#include "../../../include/math/linear/vec4.h"
#include "../../../include/math/math.h"

namespace Physics2D
{
	Vec4::Vec4()
	{
		clear();
	}
	Vec4::Vec4(__m128 d) : data(d)
	{
	}
	Vec4::Vec4(const real& fillNumber)
	{
		fill(fillNumber);
	}
	Vec4::Vec4(const real& _x, const real& _y, const real& _z, const real& _w)
		: data(_mm_setr_ps(_x, _y, _z, _w))
	{
	}

	Vec4& Vec4::operator=(const Vec4& copy)
	{
		if (&copy != this)
			set(copy);
		return *this;
	}

	Vec4& Vec4::operator=(const Vec3& xyz)
	{
		set(xyz);
		return *this;
	}

	Vec4::Vec4(const Vec3& xyz)
	{
		set(xyz);
	}

	Vec4 Vec4::operator+(const Vec4& rhs) const
	{
		return { _mm_add_ps(data, rhs.data) };
	}

	Vec4 Vec4::operator-(const Vec4& rhs) const
	{
		return { _mm_sub_ps(data, rhs.data) };
	}
	Vec4 Vec4::operator-() const
	{
		return { _mm_mul_ps(data, _mm_set_ps1(-1)) };
	}
	Vec4 Vec4::operator*(const real& factor) const
	{
		return { _mm_mul_ps(data, _mm_set_ps1(factor)) };
	}

	Vec4 Vec4::operator/(const real& factor) const
	{
		checkZero(factor);
		return { _mm_div_ps(data, _mm_set_ps1(factor)) };
	}

	Vec4& Vec4::operator+=(const Vec4& rhs)
	{
		_mm_store_ps(f, _mm_add_ps(data, rhs.data));
		return *this;
	}

	Vec4& Vec4::operator-=(const Vec4& rhs)
	{
		_mm_store_ps(f, _mm_sub_ps(data, rhs.data));
		return *this;
	}

	Vec4& Vec4::operator*=(const real& factor)
	{
		_mm_store_ps(f, _mm_mul_ps(data, _mm_set_ps1(factor)));
		return *this;
	}

	Vec4& Vec4::operator/=(const real& factor)
	{
		checkZero(factor);
		_mm_store_ps(f, _mm_div_ps(data, _mm_set_ps1(factor)));
		return *this;
	}

	Vec4& Vec4::fill(const real& number)
	{
		_mm_store_ps(f, _mm_set_ps1(number));
		return *this;
	}

	Vec4& Vec4::set(const real& _x, const real& _y, const real& _z, const real& _w)
	{
		_mm_store_ps(f, _mm_setr_ps(_x, _y, _z, _w));
		return *this;
	}


	Vec4& Vec4::set(const Vec4& other)
	{
		_mm_store_ps(f, other.data);
		return *this;
	}

	Vec4& Vec4::set(__m128 d)
	{
		_mm_store_ps(f, d);
		return *this;
	}

	Vec4& Vec4::set(const Vec3& xyz)
	{
		_mm_store_ps(f, xyz.data);
		w = 0.0f;
		return *this;
	}

	Vec4& Vec4::set(const Vec3& xyz, const real& _w)
	{
		_mm_store_ps(f, xyz.data);
		w = _w;
		return *this;
	}

	Vec4& Vec4::clear()
	{
		_mm_store_ps(f, _mm_setzero_ps());
		return *this;
	}

	Vec4& Vec4::negate()
	{
		_mm_store_ps(f, _mm_mul_ps(data, _mm_set_ps1(-1)));
		return *this;
	}

	Vec4& Vec4::normalize()
	{
		_mm_store_ps(f, _mm_mul_ps(data, _mm_rsqrt_ps(_mm_dp_ps(data, data, 0xff))));
		return *this;
	}

	real Vec4::magnitudeSquare() const
	{
		return dotProduct(*this, *this);
	}

	real Vec4::magnitude() const
	{
		return sqrt(magnitudeSquare());
	}

	Vec4 Vec4::normal() const
	{
		return Vec4(*this).normalize();
	}

	Vec4 Vec4::negative() const
	{
		return Vec4(_mm_mul_ps(data, _mm_set_ps1(-1)));
	}

	bool Vec4::equal(const Vec4& rhs) const
	{
		return realEqual(x, rhs.x) && realEqual(y, rhs.y)
			&& realEqual(z, rhs.z) && realEqual(w, rhs.w);
	}

	bool Vec4::fuzzyEqual(const Vec4& rhs, const real& epsilon) const
	{
		return (*this - rhs).magnitudeSquare() < epsilon;
	}

	bool Vec4::isOrigin(const real& epsilon) const
	{
		return fuzzyEqual({ 0, 0, 0 , 0 }, epsilon);
	}

	Vec4& Vec4::swap(Vec4& other)
	{
		__m128 temp = data;
		_mm_store_ps(f, other.data);
		_mm_store_ps(other.f, temp);
		return *this;
	}

	real Vec4::dot(const Vec4& rhs) const
	{
		return dotProduct(*this, rhs);
	}

	real Vec4::dotProduct(const Vec4& lhs, const Vec4& rhs)
	{
		//https://stackoverflow.com/questions/37879678/dot-product-performance-with-sse-instructions
		__m128 temp = _mm_dp_ps(lhs.data, rhs.data, 0xff);
		return reinterpret_cast<float*>(&temp)[0];
	}

	Vec4 Vec4::lerp(const Vec4& a, const Vec4& b, const real& t)
	{
		return a + (b - a) * t;
	}

	Vec3 Vec4::xyz() const
	{
		return Vec3(x, y, z);
	}
}
