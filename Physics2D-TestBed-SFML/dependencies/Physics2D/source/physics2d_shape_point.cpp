#include "physics2d_shape_point.h"
namespace Physics2D
{
	Point::Point()
	{
		m_type = Type::Point;
	}

	Vec2 Point::position() const
	{
		return m_position;
	}

	void Point::scale(const real& factor)
	{
		m_position *= factor;
	}
	bool Point::contains(const Vec2& point, const real& epsilon)
	{
		return (m_position - point).magnitudeSquare() < epsilon;
	}
	void Point::setPosition(const Vec2& pos)
	{
		m_position = pos;
	}

	Vec2 Point::center()const
	{
		return m_position;
	}
}