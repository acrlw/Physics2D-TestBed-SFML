#include "../../../include/math/linear/vec3.h"
#include "../../../include/math/math.h"

namespace Physics2D
{
	Vec3::Vec3()
	{
		clear();
	}

	Vec3::Vec3(const real& x, const real& y, const real& z) :
		data(_mm_setr_ps(x, y, z, 0))
	{
	}

	Vec3::Vec3(const real& fillNumber)
	{
		fill(fillNumber);
	}

	Vec3::Vec3(const Vec3& copy)
	{
		set(copy);
	}

	Vec3::Vec3(__m128 d) : data(d)
	{

	}

	Vec3 Vec3::operator+(const Vec3& rhs) const
	{
		return { _mm_add_ps(data, rhs.data) };
	}

	Vec3 Vec3::operator-(const Vec3& rhs) const
	{
		return { _mm_sub_ps(data, rhs.data) };
	}

	Vec3 Vec3::operator-() const
	{
		return { _mm_mul_ps(data, _mm_set_ps1(-1)) };
	}

	Vec3 Vec3::operator*(const real& factor) const
	{
		return { _mm_mul_ps(data, _mm_set_ps1(factor)) };
	}

	Vec3 Vec3::operator/(const real& factor) const
	{
		checkZero(factor);
		return { _mm_div_ps(data, _mm_set_ps1(factor)) };
	}

	Vec3& Vec3::operator+=(const Vec3& rhs)
	{
		_mm_store_ps(f, _mm_add_ps(data, rhs.data));
		return *this;
	}

	Vec3& Vec3::operator-=(const Vec3& rhs)
	{
		_mm_store_ps(f, _mm_sub_ps(data, rhs.data));
		return *this;
	}

	Vec3& Vec3::operator*=(const real& factor)
	{
		_mm_store_ps(f, _mm_mul_ps(data, _mm_set_ps1(factor)));
		return *this;
	}

	Vec3& Vec3::operator/=(const real& factor)
	{
		checkZero(factor);
		_mm_store_ps(f, _mm_div_ps(data, _mm_set_ps1(factor)));
		return *this;
	}

	Vec3& Vec3::fill(const real& number)
	{
		data = _mm_set_ps1(number);
		w = 0.0f;
		return *this;
	}

	Vec3& Vec3::set(const real& _x, const real& _y, const real& _z)
	{
		_mm_store_ps(f, _mm_setr_ps(_x, _y, _z, 0));
		return *this;
	}

	Vec3& Vec3::set(const Vec3& other)
	{
		_mm_store_ps(f, other.data);
		return *this;
	}

	Vec3& Vec3::set(__m128 d)
	{
		_mm_store_ps(f, d);
		w = 0.0f;
		return *this;
	}

	Vec3& Vec3::clear()
	{
		_mm_store_ps(f, _mm_setzero_ps());
		return *this;
	}

	Vec3 Vec3::negative() const
	{
		return { _mm_mul_ps(data, _mm_set_ps1(-1)) };
	}
	Vec3& Vec3::negate()
	{
		_mm_store_ps(f, _mm_mul_ps(data, _mm_set_ps1(-1)));
		return *this;
	}

	real Vec3::magnitudeSquare() const
	{
		return dotProduct(*this, *this);
	}

	real Vec3::magnitude() const
	{
		return sqrt(magnitudeSquare());
	}

	Vec3& Vec3::normalize()
	{
		_mm_store_ps(f, _mm_mul_ps(data, _mm_rsqrt_ps(_mm_dp_ps(data, data, 0xff))));
		return *this;
	}

	Vec3 Vec3::normal() const
	{
		return Vec3(*this).normalize();
	}

	bool Vec3::equal(const Vec3& rhs) const
	{
		return realEqual(x, rhs.x) && realEqual(y, rhs.y) && realEqual(z, rhs.z);
	}

	bool Vec3::fuzzyEqual(const Vec3& rhs, const real& epsilon) const
	{
		return (*this - rhs).magnitudeSquare() < epsilon;
	}
	bool Vec3::isOrigin(const real& epsilon) const
	{
		return fuzzyEqual({ 0, 0, 0 }, epsilon);
	}
	Vec3& Vec3::swap(Vec3& other)
	{
		__m128 temp = data;
		_mm_store_ps(f, other.data);
		_mm_store_ps(other.f, temp);
		return *this;
	}

	real Vec3::dot(const Vec3& rhs) const
	{
		return dotProduct(*this, rhs);
	}

	Vec3 Vec3::lerp(const Vec3& b, const real& t)
	{
		return lerp(*this, b, t);
	}

	Vec3& Vec3::cross(const Vec3& rhs)
	{
		set(crossProduct(*this, rhs));
		return *this;
	}

	real Vec3::dotProduct(const Vec3& lhs, const Vec3& rhs)
	{
		//https://stackoverflow.com/questions/37879678/dot-product-performance-with-sse-instructions
		__m128 temp = _mm_dp_ps(lhs.data, rhs.data, 0xff);
		return reinterpret_cast<float*>(&temp)[0];
	}

	Vec3 Vec3::crossProduct(const Vec3& lhs, const Vec3& rhs)
	{
		//lhs: x y z
		//rhs: x y z


		//we store as:
		//x   y   z   w
		//0   1   2   3

		//step 1.lhs: y z x
		//            * * *
		//       rhs: z x y

		//y z x w
		//1 2 0 3

		//roll to fit param order
		//w x y z
		//3 0 1 2
		//_MM_SHUFFLE(fp3, fp2, fp1, fp0)
		__m128 l1 = _mm_shuffle_ps(lhs.data, lhs.data, _MM_SHUFFLE(3, 0, 2, 1));

		//z x y w
		//2 0 1 3
		//roll to fit param order
		//w y x z
		//3 1 0 2
		//_MM_SHUFFLE(fp3, fp2, fp1, fp0)
		__m128 r1 = _mm_shuffle_ps(rhs.data, rhs.data, _MM_SHUFFLE(3, 1, 0, 2));

		//step 2.lhs: z x y
		//            * * *
		//       rhs: y z x

		//z x y w
		__m128 l2 = _mm_shuffle_ps(lhs.data, lhs.data, _MM_SHUFFLE(3, 1, 0, 2));
		//y z x w
		__m128 r2 = _mm_shuffle_ps(rhs.data, rhs.data, _MM_SHUFFLE(3, 0, 2, 1));

		//3._mm_sub_ps
		return Vec3(_mm_sub_ps(_mm_mul_ps(l1, r1), _mm_mul_ps(l2, r2)));
	}

	Vec3 Vec3::lerp(const Vec3& a, const Vec3& b, const real& t)
	{
		return a + (b - a) * t;
	}

	Vec3& Vec3::operator=(const Vec3& copy)
	{
		if (&copy != this)
			set(copy);
		return *this;
	}
}
