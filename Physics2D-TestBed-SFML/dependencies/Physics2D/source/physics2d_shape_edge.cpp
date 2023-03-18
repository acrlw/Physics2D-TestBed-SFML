#include "physics2d_shape_edge.h"
#include "physics2d_geometry_algorithm_2d.h"
namespace Physics2D
{
	Edge::Edge()
	{
		m_type = Type::Edge;
	}

	void Edge::set(const Vec2& start, const Vec2& end)
	{
		m_startPoint = start;
		m_endPoint = end;
		m_normal = (m_endPoint - m_startPoint).perpendicular().normal().negate();
	}

	void Edge::setStartPoint(const Vec2& start)
	{
		m_startPoint = start;
	}

	void Edge::setEndPoint(const Vec2& end)
	{
		m_endPoint = end;
	}

	Vec2 Edge::startPoint() const
	{
		return m_startPoint;
	}

	Vec2 Edge::endPoint() const
	{
		return m_endPoint;
	}

	void Edge::scale(const real& factor)
	{
		m_startPoint *= factor;
		m_endPoint *= factor;
	}

	bool Edge::contains(const Vec2& point, const real& epsilon)
	{
		return GeometryAlgorithm2D::isPointOnSegment(m_startPoint, m_endPoint, point);
	}

	Vec2 Edge::center()const
	{
		return (m_startPoint + m_endPoint) / 2.0f;
	}

	Vec2 Edge::normal() const
	{
		return m_normal;
	}

	void Edge::setNormal(const Vec2& normal)
	{
		m_normal = normal;
	}
}