#include "../../../include/geometry/shape/curve.h"
namespace Physics2D
{
	Curve::Curve()
	{
		m_type = Type::Curve;
	}

	void Curve::set(const Vec2& start, const Vec2& control1, const Vec2& control2, const Vec2& end)
	{
		m_startPoint = start;
		m_control1 = control1;
		m_control2 = control2;
		m_endPoint = end;
	}

	Vec2 Curve::startPoint() const
	{
		return m_startPoint;
	}

	void Curve::setStartPoint(const Vec2& startPoint)
	{
		m_startPoint = startPoint;
	}

	Vec2 Curve::control1() const
	{
		return m_control1;
	}

	void Curve::setControl1(const Vec2& control1)
	{
		m_control1 = control1;
	}

	Vec2 Curve::control2() const
	{
		return m_control2;
	}

	void Curve::setControl2(const Vec2& control2)
	{
		m_control2 = control2;
	}

	Vec2 Curve::endPoint() const
	{
		return m_endPoint;
	}

	void Curve::setEndPoint(const Vec2& endPoint)
	{
		m_endPoint = endPoint;
	}

	void Curve::scale(const real& factor)
	{
		m_startPoint *= factor;
		m_control1 *= factor;
		m_control2 *= factor;
		m_endPoint *= factor;
	}
	bool Curve::contains(const Vec2& point, const real& epsilon)
	{
		return false;
	}
	Vec2 Curve::center() const
	{
		return Vec2();
	}

}