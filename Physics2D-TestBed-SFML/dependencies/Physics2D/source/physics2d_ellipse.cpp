#include "physics2d_ellipse.h"
namespace Physics2D
{
	Ellipse::Ellipse(const real& width, const real& height) : m_width(width), m_height(height)
	{
		m_type = Type::Ellipse;
	}

	void Ellipse::set(const Vector2& leftTop, const Vector2& rightBottom)
	{
		m_width = std::fabs(rightBottom.x - leftTop.x);
		m_height = std::fabs(rightBottom.y - leftTop.y);
	}

	void Ellipse::set(const real& width, const real& height)
	{
		m_width = width;
		m_height = height;
	}

	void Ellipse::setWidth(const real& width)
	{
		m_width = width;
	}

	void Ellipse::setHeight(const real& height)
	{
		m_height = height;
	}

	void Ellipse::scale(const real& factor)
	{
		m_width *= factor;
		m_height *= factor;
	}

	bool Ellipse::contains(const Vector2& point, const real& epsilon)
	{
		real a = A();
		real b = B();
		assert(!realEqual(a, 0) && !realEqual(b, 0));
		return (point.x - a) * (point.x - a) / (a * a) + (point.y - b) * (point.y - b) / (b * b) <= 1.0f;
	}

	Vector2 Ellipse::center()const
	{
		return Vector2();
	}

	real Ellipse::width() const
	{
		return m_width;
	}

	real Ellipse::height() const
	{
		return m_height;
	}

	real Ellipse::A() const
	{
		return m_width / 2.0f;
	}

	real Ellipse::B() const
	{
		return m_height / 2.0f;
	}

	real Ellipse::C() const
	{
		real a = A();
		real b = B();
		return sqrt(a * a - b * b);
	}
}