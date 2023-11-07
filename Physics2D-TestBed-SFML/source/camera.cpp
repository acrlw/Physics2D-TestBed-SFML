#include "camera.h"

#include <iostream>

#include "render.h"

namespace Physics2D
{
	Camera::Camera()
	{

	}

	void Camera::render(sf::RenderWindow& window)
	{
		if (m_visible)
		{
			assert(m_world != nullptr);

			real inv_dt = 1.0f / m_deltaTime;

			real scale = m_targetMeterToPixel - m_meterToPixel;
			bool isZooming = !realEqual(m_meterToPixel, m_targetMeterToPixel);
			
			
			if (m_smoothZoom && !(std::fabs(scale) < 0.1f || m_meterToPixel < 1.0f))
				m_meterToPixel -= (1.0f - std::exp(m_restitution * inv_dt)) * scale;
			else
				m_meterToPixel = m_targetMeterToPixel;

			m_pixelToMeter = 1.0f / m_meterToPixel;

			if (isZooming && !m_preScreenMousePos.isOrigin())
				m_transform += (screenToWorld(m_preScreenMousePos) - m_preWorldMousePos) * m_meterToPixel;


			if (m_targetBody != nullptr)
			{
				Vector2 real_origin(m_origin.x + m_transform.x, m_origin.y - m_transform.y);
				Vector2 target(-(m_targetBody->position().x + m_targetBody->shape()->center().x),
				               m_targetBody->position().y + m_targetBody->shape()->center().y);
				target = worldToScreen(target) - real_origin;

				Vector2 c = target - m_transform;

				switch (m_easingType)
				{
				case EasingType::Exponential:
					{
						if (c.lengthSquare() < 0.1f)
							m_transform = target;
						else
							m_transform -= (1.0f - std::exp(m_restitution * inv_dt)) * c;
						break;
					}
				case EasingType::Lerp:
					{
						if (c.lengthSquare() < 0.1f)
							m_transform = target;
						else
							m_transform += 0.02f * c;
						break;
					}
				case EasingType::Uniform:
					{
						break;
					}
				}
			}


			//draw background
			window.clear(sf::Color(50, 50, 50));




			if (m_aabbVisible)
			{
				for (auto iter = m_world->bodyList().begin(); iter != m_world->bodyList().end(); ++iter)
				{
					if ((*iter).get() != nullptr)
						RenderSFMLImpl::renderAABB(window, *this, (*iter)->aabb(), sf::Color::Cyan);
				}
			}
			if (m_treeVisible)
			{
				drawTree(m_tree->rootIndex(), window);
			}
			if (m_uniformGridVisible)
			{
				for (auto&& elem : m_grid->m_cellsToBodies)
				{
					Vector2 topLeft(static_cast<real>(elem.first.x) * m_grid->cellWidth() - m_grid->width() * 0.5f,
					                static_cast<real>(elem.first.y) * m_grid->cellHeight() - m_grid->height() * 0.5f);
					AABB cell(topLeft, m_grid->cellWidth(), m_grid->cellHeight());
					//cell.expand(-0.05f);

					RenderSFMLImpl::renderAABB(window, *this, cell, sf::Color::Cyan);
				}
			}
			if (m_gridScaleLineVisible)
			{
				//draw axis

				sf::Color color = sf::Color::Green;
				color.a = 250;

				Container::Vector<Vector2> axisPoints;
				axisPoints.reserve(m_axisPointCount * 2 + 1);

				for (int i = -m_axisPointCount; i <= m_axisPointCount; ++i)
				{
					axisPoints.emplace_back(Vector2(0, static_cast<real>(i)));
					axisPoints.emplace_back(Vector2(static_cast<real>(i), 0));
				}
				

				//draw grid
				drawGridScaleLine(window);
				RenderSFMLImpl::renderPoints(window, *this, axisPoints, color);
				color.a = 140;
				RenderSFMLImpl::renderLine(window, *this, Vector2(0.0f, static_cast<real>(-m_axisPointCount)),
				                           Vector2(0.0f, static_cast<real>(m_axisPointCount)), color);
				RenderSFMLImpl::renderLine(window, *this, Vector2(static_cast<real>(-m_axisPointCount), 0.0f),
				                           Vector2(static_cast<real>(m_axisPointCount), 0.0f), color);
			}
			if (m_bodyVisible)
			{
				sf::Color color = sf::Color::Green;

				for (auto& body : m_world->bodyList())
				{
					ShapePrimitive primitive;
					primitive.shape = body->shape();
					primitive.transform.rotation = body->rotation();
					primitive.transform.position = body->position();
					if (body->sleep())
						color = sf::Color(100, 100, 100);
					else
						color = RenderConstant::Green;
					if (body->type() == Body::BodyType::Static)
						color = RenderConstant::Teal;
					RenderSFMLImpl::renderShape(window, *this, primitive, color);

					if (m_centerVisible)
					{
						RenderSFMLImpl::renderAngleLine(window, *this, primitive, sf::Color::Green);
					}

					if (body->type() != Body::BodyType::Static)
					{
						if (m_bodyVelocity)
						{
							RenderSFMLImpl::renderArrow(window, *this, primitive.transform.position,
							                            primitive.transform.position + body->velocity(),
							                            RenderConstant::Orange, 0.2f);
						}

						if (m_bodyVelocityMagnitude)
						{
							std::string str = std::format("{:.2f}", body->velocity().length());
							const Vector2 offset(-0.01f, 0.01f);
							RenderSFMLImpl::renderText(window, *this, primitive.transform.position + offset, *m_font,
							                           str, RenderConstant::Orange, 16);
						}

						if (m_bodyVelocityNormal)
						{
							Vector2 vel = body->velocity();
							const real length = vel.length();
							if (!realEqual(length, 0.0f))
								RenderSFMLImpl::renderArrow(window, *this, primitive.transform.position,
								                            primitive.transform.position + vel / length,
								                            RenderConstant::Orange, 0.2f);
						}
					}
				}
			}
			if (m_jointVisible)
			{
				for (auto iter = m_world->jointList().begin(); iter != m_world->jointList().end(); ++iter)
				{
					if ((*iter)->active())
						RenderSFMLImpl::renderJoint(window, *this, (*iter).get(), sf::Color::Green);
				}
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

	bool& Camera::gridScaleLineVisible()
	{
		return m_gridScaleLineVisible;
	}

	int Camera::axisPointCount() const
	{
		return m_axisPointCount;
	}

	void Camera::setAxisPointCount(int count)
	{
		m_axisPointCount = count;
	}

	real Camera::meterToPixel() const
	{
		return m_meterToPixel;
	}

	void Camera::setTargetMeterToPixel(const real& meterToPixel)
	{
		if (meterToPixel < 1.0f)
		{
			m_targetMeterToPixel = 1.0f;
			m_targetPixelToMeter = 1.0f;
			return;
		}
		m_targetMeterToPixel = meterToPixel;
		m_targetPixelToMeter = 1.0f / meterToPixel;
	}

	real Camera::pixelToMeter() const
	{
		return m_pixelToMeter;
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


	Camera::Viewport Camera::viewport() const
	{
		return m_viewport;
	}

	void Camera::setViewport(const Viewport& viewport)
	{
		m_viewport = viewport;
		m_origin.set((m_viewport.topLeft.x + m_viewport.bottomRight.x) * (0.5),
		             (m_viewport.topLeft.y + m_viewport.bottomRight.y) * (0.5));
	}

	Vector2 Camera::worldToScreen(const Vector2& pos) const
	{
		Vector2 real_origin(m_origin.x + m_transform.x, m_origin.y - m_transform.y);
		return Vector2(real_origin.x + pos.x * m_meterToPixel, real_origin.y - pos.y * m_meterToPixel);
	}

	Vector2 Camera::screenToWorld(const Vector2& pos) const
	{
		Vector2 real_origin(m_origin.x + m_transform.x, m_origin.y - m_transform.y);
		Vector2 result = pos - real_origin;
		result.y = -result.y;
		result *= m_pixelToMeter;
		return result;
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


	bool& Camera::centerVisible()
	{
		return m_centerVisible;
	}

	bool& Camera::contactVisible()
	{
		return m_contactVisible;
	}

	bool& Camera::uniformGridVisible()
	{
		return m_uniformGridVisible;
	}

	bool& Camera::contactImpulseVisible()
	{
		return m_contactImpulseVisible;
	}

	bool& Camera::contactImpulseMagnitude()
	{
		return m_contactImpulseMagnitude;
	}

	bool& Camera::contactFrictionVisible()
	{
		return m_contactFrictionVisible;
	}

	bool& Camera::contactFrictionMagnitude()
	{
		return m_contactFrictionMagnitude;
	}

	void Camera::setFont(sf::Font* font)
	{
		m_font = font;
	}

	sf::Font* Camera::font()
	{
		return m_font;
	}

	bool& Camera::bodyVelocity()
	{
		return m_bodyVelocity;
	}

	bool& Camera::bodyVelocityNormal()
	{
		return m_bodyVelocityNormal;
	}

	bool& Camera::bodyVelocityMagnitude()
	{
		return m_bodyVelocityMagnitude;
	}

	bool& Camera::coordinateScale()
	{
		return m_drawCoordinateScale;
	}

	bool& Camera::smoothZoom()
	{
		return m_smoothZoom;
	}

	ContactMaintainer* Camera::maintainer() const
	{
		return m_maintainer;
	}

	void Camera::setContactMaintainer(ContactMaintainer* maintainer)
	{
		m_maintainer = maintainer;
	}

	Camera::EasingType Camera::easingType() const
	{
		return m_easingType;
	}

	void Camera::setEasingType(EasingType type)
	{
		m_easingType = type;
	}

	UniformGrid* Camera::uniformGrid() const
	{
		return m_grid;
	}

	void Camera::setUniformGrid(UniformGrid* grid)
	{
		m_grid = grid;
	}

	real Camera::defaultMeterToPixel() const
	{
		return m_defaultMeterToPixel;
	}

	void Camera::setDefaultMeterToPixel(const real& number)
	{
		m_defaultMeterToPixel = number;
	}

	void Camera::setPreScreenMousePos(const Vector2& pos)
	{
		m_preScreenMousePos = pos;
		m_preWorldMousePos = screenToWorld(pos);
	}


	void Camera::drawTree(int nodeIndex, sf::RenderWindow& window)
	{
		if (nodeIndex == -1)
			return;
		//std::cout << "tree size:" << m_tree->tree().size() << std::endl;
		drawTree(m_tree->tree()[nodeIndex].leftIndex, window);
		drawTree(m_tree->tree()[nodeIndex].rightIndex, window);

		AABB aabb = m_tree->tree()[nodeIndex].aabb;
		//aabb.expand(0.5);
		if (!m_tree->tree()[nodeIndex].isLeaf())
			RenderSFMLImpl::renderAABB(window, *this, aabb, sf::Color::Cyan);
	}

	void Camera::drawContacts(sf::RenderWindow& window)
	{
		sf::Color pink = RenderConstant::Pink;
		sf::Color yellow = sf::Color::Yellow;
		pink.a = 200;
		yellow.a = 200;

		for (auto iter = m_maintainer->m_contactTable.begin(); iter != m_maintainer->m_contactTable.end(); ++iter)
		{
			for (auto& elem : iter->second)
			{
				const Vector2 realA = elem.bodyA->toWorldPoint(elem.vcp.localA);
				const Vector2 realB = elem.bodyB->toWorldPoint(elem.vcp.localB);
				if (m_contactImpulseVisible)
					RenderSFMLImpl::renderArrow(window, *this, realB,
					                            realB - elem.vcp.normal * elem.vcp.accumulatedNormalImpulse,
					                            RenderConstant::Cyan, 0.2f);

				if (m_contactImpulseMagnitude)
					RenderSFMLImpl::renderFloat(window, *this, realB, *m_font, elem.vcp.accumulatedNormalImpulse,
					                            RenderConstant::Cyan, 16, elem.vcp.normal * 0.55f);

				if (m_contactFrictionVisible)
					RenderSFMLImpl::renderArrow(window, *this, elem.bodyB->toWorldPoint(elem.vcp.localB),
					                            elem.bodyB->toWorldPoint(elem.vcp.localB) - elem.vcp.tangent * elem.vcp.
					                            accumulatedTangentImpulse, RenderConstant::Yellow, 0.2f);

				if (m_contactFrictionMagnitude)
				{
					const Vector2 offset(0.05f, -0.05f);
					RenderSFMLImpl::renderFloat(window, *this, realB, *m_font, elem.vcp.accumulatedTangentImpulse,
					                            yellow, 16, offset);
				}

				RenderSFMLImpl::renderPoint(window, *this, realA, pink);
				RenderSFMLImpl::renderPoint(window, *this, realB, yellow);
			}
		}
	}

	void Camera::drawGridScaleLine(sf::RenderWindow& window)
	{
		sf::Color thick = RenderConstant::DarkGreen;
		thick.a = 160;
		sf::Color thin = RenderConstant::DarkGreen;
		thin.a = 60;

		const bool fineEnough = m_meterToPixel > 125;

		Container::Vector<std::pair<Vector2, Vector2>> lines;
		lines.reserve(m_axisPointCount * 2);

		int h = 1;

		if (m_meterToPixel < 30)
			h = 10;
		else if (m_meterToPixel < 60)
			h = 5;
		else if (m_meterToPixel < 120)
			h = 2;

		sf::Color color = sf::Color::Green;
		color.a = 80;
		for (int i = -m_axisPointCount; i <= m_axisPointCount; i += h)
		{
			Vector2 p1 = {static_cast<real>(i), static_cast<real>(m_axisPointCount)};
			Vector2 p2 = {static_cast<real>(i), static_cast<real>(-m_axisPointCount)};
			lines.emplace_back(std::make_pair(p1, p2));

			p1.set(static_cast<real>(-m_axisPointCount), static_cast<real>(i));
			p2.set(static_cast<real>(m_axisPointCount), static_cast<real>(i));
			lines.emplace_back(std::make_pair(p1, p2));

			//draw number
			if(!m_drawCoordinateScale)
				continue;

			std::string str = std::format("{}", i);
			RenderSFMLImpl::renderText(window, *this, Vector2(static_cast<real>(i), 0.0f), *m_font, str, color, 16, { -0.25f, -0.25f });
			if (i == 0)
				continue;
			RenderSFMLImpl::renderText(window, *this, Vector2(0.0f, static_cast<real>(i)), *m_font, str, color, 16, { -0.25f, -0.25f });

		}
		RenderSFMLImpl::renderLines(window, *this, lines, thick);


		if (fineEnough)
		{
			lines.clear();
			lines.reserve(m_axisPointCount);

			int slice = 50;
			if (m_meterToPixel < 250)
				slice = 2;
			else if (m_meterToPixel < 800)
				slice = 10;
			else if (m_meterToPixel < 2000)
				slice = 20;

			const real inv = 1.0f / static_cast<real>(slice);
			for (int i = -m_axisPointCount, j = -m_axisPointCount + h; i < m_axisPointCount; i += h, j += h)
			{
				for (int k = 1; k < slice; ++k)
				{
					real index = static_cast<real>(k) * inv;

					Vector2 p1 = {static_cast<real>(i) + index, static_cast<real>(m_axisPointCount)};
					Vector2 p2 = {static_cast<real>(i) + index, static_cast<real>(-m_axisPointCount)};
					lines.emplace_back(std::make_pair(p1, p2));

					p1.set(static_cast<real>(-m_axisPointCount), static_cast<real>(i) + index);
					p2.set(static_cast<real>(m_axisPointCount), static_cast<real>(i) + index);
					lines.emplace_back(std::make_pair(p1, p2));

				}
			}
			RenderSFMLImpl::renderLines(window, *this, lines, thin);
		}
	}

	real Camera::Viewport::width()
	{
		return bottomRight.x - topLeft.x;
	}

	real Camera::Viewport::height()
	{
		return bottomRight.y - topLeft.y;
	}

	Vector2 Camera::Viewport::center() const
	{
		return bottomRight * 0.5f;
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
