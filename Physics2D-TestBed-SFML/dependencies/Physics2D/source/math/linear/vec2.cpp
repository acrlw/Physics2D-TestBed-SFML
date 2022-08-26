#include "../../../include/math/linear/vec2.h"
#include "../../../include/math/math.h"
namespace Physics2D
{
	Vec2::Vec2() : x(0), y(0)
	{
	}
	Vec2::Vec2(const real& fillNumber) : x(fillNumber), y(fillNumber)
	{
	}
	Vec2::Vec2(const real& _x, const real& _y) : x(_x), y(_y)
	{
	}
	Vec2::Vec2(const Vec2& copy) : x(copy.x), y(copy.y)
	{
	}

	real& Vec2::operator[](uint8_t index)
	{
		return f[index];
	}

	Vec2 Vec2::operator+(const Vec2& rhs) const
	{
		return Vec2(x + rhs.x, y + rhs.y);
	}
	Vec2& Vec2::operator*=(const real& factor)
	{
		x *= factor;
		y *= factor;
		return *this;
	}
	Vec2 Vec2::operator-(const Vec2& rhs) const
	{
		return Vec2(x - rhs.x, y - rhs.y);
	}
	Vec2 Vec2::operator-()const
	{
		return Vec2(-x, -y);
	}
	Vec2& Vec2::operator+=(const Vec2& rhs)
	{
		x += rhs.x;
		y += rhs.y;
		return *this;
	}
	Vec2 Vec2::operator/(const real& factor) const
	{
		assert(!realEqual(factor, 0));
		return Vec2(x / factor, y / factor);
	}

	Vec2& Vec2::operator-=(const Vec2& rhs)
	{
		x -= rhs.x;
		y -= rhs.y;
		return *this;
	}

	Vec2& Vec2::operator/=(const real& factor)
	{
		checkZero(factor);
		x /= factor;
		y /= factor;
		return *this;
	}

	bool Vec2::operator==(const Vec2& rhs) const
	{
		return realEqual(x, rhs.x) && realEqual(y, rhs.y);
	}

	bool Vec2::operator!=(const Vec2& rhs) const
	{
		return !realEqual(x, rhs.x) || !realEqual(y, rhs.y);
	}

	real Vec2::magnitudeSquare() const
	{
		return x * x + y * y;
	}

	real Vec2::magnitude() const
	{
		return std::sqrt(magnitudeSquare());
	}

	real Vec2::theta() const
	{
		return arctanx(y, x);
	}

	Vec2& Vec2::fill(const real& fillNumber)
	{
		x = fillNumber;
		y = fillNumber;
		return *this;
	}

	Vec2& Vec2::set(const real& _x, const real& _y)
	{
		x = _x;
		y = _y;
		return *this;
	}

	Vec2& Vec2::set(const Vec2& copy)
	{
		x = copy.x;
		y = copy.y;
		return *this;
	}

	Vec2& Vec2::clear()
	{
		x = 0.0f;
		y = 0.0f;
		return *this;
	}

	Vec2& Vec2::negate()
	{
		x *= -1.0f;
		y *= -1.0f;
		return *this;
	}

	Vec2& Vec2::swap(Vec2& other) noexcept
	{
		realSwap(x, other.x);
		realSwap(y, other.y);
		return *this;
	}

	Vec2& Vec2::normalize()
	{
		const real length_inv = fastInverseSqrt<real>(magnitudeSquare());
		x *= length_inv;
		y *= length_inv;
		return *this;
	}

	Vec2 Vec2::normal() const
	{
		return Vec2(*this).normalize();
	}

	Vec2 Vec2::negative() const
	{
		return Vec2(-x, -y);
	}

	bool Vec2::equal(const Vec2& rhs) const
	{
		return realEqual(x, rhs.x) && realEqual(y, rhs.y);
	}

	bool Vec2::fuzzyEqual(const Vec2& rhs, const real& epsilon)const
	{
		return fuzzyRealEqual(x, rhs.x, epsilon) && fuzzyRealEqual(y, rhs.y, epsilon);
	}

	bool Vec2::isOrigin(const real& epsilon) const
	{
		return fuzzyEqual({ 0, 0 }, epsilon);
	}

	real Vec2::dot(const Vec2& rhs) const
	{
		return x * rhs.x + y * rhs.y;
	}

	real Vec2::cross(const Vec2& rhs) const
	{
		return x * rhs.y - y * rhs.x;
	}

	Vec2 Vec2::perpendicular() const
	{
		return Vec2(-y, x);
	}

	real Vec2::dotProduct(const Vec2& lhs, const Vec2& rhs)
	{
		return lhs.x * rhs.x + lhs.y * rhs.y;
	}

	real Vec2::crossProduct(const Vec2& lhs, const Vec2& rhs)
	{
		return lhs.x * rhs.y - lhs.y * rhs.x;
	}

	real Vec2::crossProduct(const real& x1, const real& y1, const real& x2, const real& y2)
	{
		return x1 * y2 - x2 * y1;
	}

	Vec2 Vec2::crossProduct(const real& lhs, const Vec2& rhs)
	{
		return Vec2(-rhs.y, rhs.x) * lhs;
	}

	Vec2 Vec2::crossProduct(const Vec2& lhs, const real& rhs)
	{
		return Vec2(lhs.y, -lhs.x) * rhs;
	}

	Vec2 Vec2::lerp(const Vec2& a, const Vec2& b, const real& t)
	{
		return a + (b - a) * t;
	}

	Vec2 Vec2::operator*(const real& factor) const
	{
		return Vec2(x * factor, y * factor);
	}

	Vec2& Vec2::operator=(const Vec2& copy)
	{
		if (&copy != this)
			set(copy);
		return *this;
	}
}
