#include "include/camera.h"
#include "include/render.h"
namespace Physics2D
{
	void Camera::render(sf::RenderWindow& window)
	{
		if (m_visible)
		{
			assert(m_world != nullptr);

			real inv_dt = 1.0 / m_deltaTime;

			real scale = m_targetMeterToPixel - m_meterToPixel;
			if (std::fabs(scale) < 0.1 || m_meterToPixel < 1.0)
				m_meterToPixel = m_targetMeterToPixel;
			else
				m_meterToPixel -= (1.0 - std::exp(m_restitution * inv_dt)) * scale;
			m_pixelToMeter = 1.0 / m_meterToPixel;

			if (m_targetBody != nullptr)
			{
				Vector2 real_origin(m_origin.x + m_transform.x, m_origin.y - m_transform.y);
				Vector2 target(-(m_targetBody->position().x + m_targetBody->shape()->center().x), m_targetBody->position().y + m_targetBody->shape()->center().y);
				target = worldToScreen(target) - real_origin;

				Vector2 c = target - m_transform;

				//lerp
				//if (c.lengthSquare() < 0.1)
				//	m_transform = target;
				//else
				//	m_transform += c * 0.02;

				//exp
				if (c.lengthSquare() < 0.1)
					m_transform = target;
				else
					m_transform -= (1.0 - std::exp(m_restitution * inv_dt)) * c;

				//real frames = m_easingTime * inv_dt;


			}


			//draw background
			window.clear(sf::Color(50, 50, 50));

			//RenderSFMLImpl::renderPoint(window, *this, Vector2(0, 0), sf::Color::Green);
			ShapePrimitive primitive;
			Circle circle(1.0f);
			primitive.shape = &circle;
			primitive.rotation = 0;
			RenderSFMLImpl::renderCircle(window, *this, primitive, sf::Color::Green);
			

			if (m_bodyVisible)
			{
				sf::Color color = sf::Color::Green;
				for (auto& body : m_world->bodyList())
				{
					ShapePrimitive primitive;
					primitive.shape = body->shape();
					primitive.rotation = body->rotation();
					primitive.transform = body->position();
					if (body->sleep())
						color = sf::Color(100, 100, 100);
					RenderSFMLImpl::renderShape(window, *this, primitive, color);
					if (m_centerVisible)
					{
						color = sf::Color(150, 150, 150);
						RenderSFMLImpl::renderPoint(window, *this, primitive.transform, color);
					}
					if (m_rotationLineVisible)
						RenderSFMLImpl::renderAngleLine(window, *this, primitive, sf::Color::Green);
				}
			}
			if (m_jointVisible)
			{
				for (auto& joint : m_world->jointList())
					if (joint->active())
						RenderSFMLImpl::renderJoint(window, *this, joint.get(), sf::Color::Green);
			}
			if (m_axisVisible)
			{
				sf::Color color = sf::Color::Green;
				color.a = 225;

				std::vector<Vector2> axisPoints;
				axisPoints.reserve(static_cast<size_t>(m_axisPointCount * 2 + 1));

				for (real i = -m_axisPointCount; i <= m_axisPointCount; i += 1.0)
				{
					axisPoints.emplace_back(Vector2(0, i));
					axisPoints.emplace_back(Vector2(i, 0));
				}
				RenderSFMLImpl::renderPoints(window, *this, axisPoints, color);
				color.a = 100;
				RenderSFMLImpl::renderLine(window, *this, Vector2(0, -m_axisPointCount), Vector2(0, m_axisPointCount), color);
				RenderSFMLImpl::renderLine(window, *this, Vector2(-m_axisPointCount, 0), Vector2(m_axisPointCount, 0), color);

			}
			if (m_aabbVisible)
			{
				for (auto [body, node] : m_dbvh->leaves())
					RenderSFMLImpl::renderAABB(window, *this, node->aabb, sf::Color::Cyan);
				for (auto& elem : m_tree->tree())
					if (elem.body != nullptr)
						RenderSFMLImpl::renderAABB(window, *this, elem.aabb, sf::Color::Cyan);
			}
			if (m_dbvhVisible)
			{
				drawDbvh(m_dbvh->root(), window);
			}
			if (m_treeVisible)
			{
				drawTree(m_tree->rootIndex(), window);
			}
			if (m_gridScaleLineVisible)
			{
				drawGridScaleLine(window);
			}
			if (m_contactVisible)
			{
				drawContacts(window);
			}
		}
	}

	bool& Camera::aabbVisible()
	{
		return m_aabbVisible;
	}

	bool& Camera::jointVisible()
	{
		return m_jointVisible;
	}

	bool& Camera::bodyVisible()
	{
		return m_bodyVisible;
	}

	bool& Camera::axisVisible()
	{
		return m_axisVisible;
	}

	bool& Camera::gridScaleLineVisible()
	{
		return m_gridScaleLineVisible;
	}

	real Camera::axisPointCount() const
	{
		return m_axisPointCount;
	}

	void Camera::setAxisPointCount(real count)
	{
		m_axisPointCount = count;
	}

	real Camera::meterToPixel() const
	{
		return m_meterToPixel;
	}

	void Camera::setMeterToPixel(const real& meterToPixel)
	{
		if (meterToPixel < 1.0)
		{
			m_targetMeterToPixel = 1.0;
			m_targetPixelToMeter = 1.0;
			return;
		}
		m_targetMeterToPixel = meterToPixel;
		m_targetPixelToMeter = 1.0f / meterToPixel;
	}

	Vector2 Camera::transform() const
	{
		return m_transform;
	}

	void Camera::setTransform(const Vector2& transform)
	{
		m_transform = transform;
	}

	void Camera::setWorld(PhysicsWorld* world)
	{
		m_world = world;
	}

	PhysicsWorld* Camera::world() const
	{
		return m_world;
	}

	Body* Camera::targetBody() const
	{
		return m_targetBody;
	}

	void Camera::setTargetBody(Body* targetBody)
	{
		m_targetBody = targetBody;
	}

	real Camera::zoomFactor() const
	{
		return m_zoomFactor;
	}

	void Camera::setZoomFactor(const real& zoomFactor)
	{
		m_zoomFactor = zoomFactor;
	}

	bool& Camera::dbvhVisible()
	{
		return m_dbvhVisible;
	}


	Camera::Viewport Camera::viewport() const
	{
		return m_viewport;
	}

	void Camera::setViewport(const Viewport& viewport)
	{
		m_viewport = viewport;
		m_origin.set((m_viewport.topLeft.x + m_viewport.bottomRight.x) * (0.5), (m_viewport.topLeft.y + m_viewport.bottomRight.y) * (0.5));
	}
	Vector2 Camera::worldToScreen(const Vector2& pos)const
	{
		Vector2 real_origin(m_origin.x + m_transform.x, m_origin.y - m_transform.y);
		return Vector2(real_origin.x + pos.x * m_meterToPixel, real_origin.y - pos.y * m_meterToPixel);
	}
	Vector2 Camera::screenToWorld(const Vector2& pos)const
	{
		Vector2 real_origin(m_origin.x + m_transform.x, m_origin.y - m_transform.y);
		Vector2 result = pos - real_origin;
		result.y = -result.y;
		result *= m_pixelToMeter;
		return result;
	}
	DBVH* Camera::dbvh() const
	{
		return m_dbvh;
	}

	void Camera::setDbvh(DBVH* dbvh)
	{
		m_dbvh = dbvh;
	}

	Tree* Camera::tree() const
	{
		return m_tree;
	}

	void Camera::setTree(Tree* tree)
	{
		m_tree = tree;
	}

	bool& Camera::visible() 
	{
		return m_visible;
	}

	bool& Camera::treeVisible()
	{
		return m_treeVisible;
	}


	real Camera::deltaTime() const
	{
		return m_deltaTime;
	}

	void Camera::setDeltaTime(const real& deltaTime)
	{
		m_deltaTime = deltaTime;
	}

	bool& Camera::rotationLineVisible()
	{
		return m_rotationLineVisible;
	}


	bool& Camera::centerVisible()
	{
		return m_centerVisible;
	}

	bool& Camera::contactVisible() 
	{
		return m_contactVisible;
	}


	ContactMaintainer* Camera::maintainer() const
	{
		return m_maintainer;
	}

	void Camera::setContactMaintainer(ContactMaintainer* maintainer)
	{
		m_maintainer = maintainer;
	}

	void Camera::drawTree(int nodeIndex, sf::RenderWindow& window)
	{
		if (nodeIndex == -1)
			return;

		drawTree(m_tree->tree()[nodeIndex].leftIndex, window);
		drawTree(m_tree->tree()[nodeIndex].rightIndex, window);

		AABB aabb = m_tree->tree()[nodeIndex].aabb;
		//aabb.expand(0.5);
		if (!m_tree->tree()[nodeIndex].isLeaf())
			RenderSFMLImpl::renderAABB(window, *this, aabb, sf::Color::Cyan);
	}
	void Camera::drawContacts(sf::RenderWindow& window)
	{
		sf::Color pink = RenderConstant::materialPink;
		sf::Color yellow = sf::Color::Yellow;
		pink.a = 128;
		yellow.a = 128;

		for (auto iter = m_maintainer->m_contactTable.begin(); iter != m_maintainer->m_contactTable.end(); ++iter)
		{
			for (auto& elem : iter->second)
			{
				RenderSFMLImpl::renderPoint(window, *this, elem.bodyA->toWorldPoint(elem.localA), pink);
				RenderSFMLImpl::renderPoint(window, *this, elem.bodyB->toWorldPoint(elem.localB), yellow);
			}
		}
	}
	void Camera::drawGridScaleLine(sf::RenderWindow& window)
	{
		sf::Color darkGreen = RenderConstant::materialDarkGreen;
		darkGreen.a = 204;
		bool fineEnough = m_meterToPixel > 180;
		real h;

		if (m_meterToPixel < 30)
			h = 10.0;
		else if (m_meterToPixel < 60)
			h = 5.0;
		else if (m_meterToPixel < 120)
			h = 2.0;
		else
			h = 1.0;

		std::vector<std::pair<Vector2, Vector2>> lines;
		lines.reserve(static_cast<size_t>(m_axisPointCount * 2));
		for (real i = -m_axisPointCount; i <= m_axisPointCount; i += h)
		{
			if (realEqual(i, 0.0))
				continue;
			Vector2 p1 = { i, m_axisPointCount };
			Vector2 p2 = { i, -m_axisPointCount };
			lines.emplace_back(std::make_pair(p1, p2));

			p1.set(-m_axisPointCount, i);
			p2.set(m_axisPointCount, i);
			lines.emplace_back(std::make_pair(p1, p2));
		}
		RenderSFMLImpl::renderLines(window, *this, lines, darkGreen);
		if (fineEnough)
		{
			if (m_meterToPixel < 400)
				h = 0.2f;
			else if (m_meterToPixel < 800)
				h = 0.1f;
			else if (m_meterToPixel < 1600)
				h = 0.05f;
			else if (m_meterToPixel < 4000)
				h = 0.02f;
			else
				h = 0.005f;

			lines.clear();
			lines.reserve(static_cast<size_t>(m_axisPointCount * 2.0f / 0.2f));
			darkGreen.a = 90;
			for (real i = -m_axisPointCount; i <= m_axisPointCount; i += h)
			{
				if (realEqual(i, 0.0f))
					continue;
				if (realEqual(i - std::floor(i), 0.0f))
					continue;
				Vector2 p1 = { i, m_axisPointCount };
				Vector2 p2 = { i, -m_axisPointCount };
				lines.emplace_back(std::make_pair(p1, p2));

				p1.set(-m_axisPointCount, i);
				p2.set(m_axisPointCount, i);
				lines.emplace_back(std::make_pair(p1, p2));
			}
			RenderSFMLImpl::renderLines(window, *this, lines, darkGreen);
		}
	}
	void Camera::drawDbvh(DBVH::Node* node, sf::RenderWindow& window)
	{
		if (node == nullptr)
			return;

		drawDbvh(node->left, window);
		drawDbvh(node->right, window);

		AABB aabb = node->aabb;
		if (!node->isLeaf())
			RenderSFMLImpl::renderAABB(window, *this, aabb, sf::Color::Cyan);

	}
	real Camera::Viewport::width()
	{
		return bottomRight.x - topLeft.x;
	}
	real Camera::Viewport::height()
	{
		return bottomRight.y - topLeft.y;
	}
	void Camera::Viewport::setWidth(const real& width)
	{
		bottomRight.x = topLeft.x + width;
	}
	void Camera::Viewport::setHeight(const real& height)
	{
		bottomRight.y = topLeft.y + height;
	}
	void Camera::Viewport::set(const real& width, const real& height)
	{
		setWidth(width);
		setHeight(height);
	}
}