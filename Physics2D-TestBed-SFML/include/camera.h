#ifndef PHYSICS2D_UTILS_CAMERA_H
#define PHYSICS2D_UTILS_CAMERA_H

#include <SFML/Graphics/CircleShape.hpp>
#include <SFML/Graphics/RenderWindow.hpp>
#include <SFML/System/Clock.hpp>
#include <SFML/Window/Event.hpp>
#include <SFML/Graphics.hpp>

#include "physics2d.h"

namespace Physics2D
{
	class Camera
	{
	public:
		enum class EasingType
		{
			Lerp,
			Exponential,
			Uniform
		};

		struct Viewport
		{
			Viewport() = default;

			Viewport(const Vector2& top_left, const Vector2& bottom_right) : topLeft(top_left),
			                                                                 bottomRight(bottom_right)
			{
			}

			Vector2 topLeft = {0, 0};
			Vector2 bottomRight = {800, 600};
			real width();
			real height();
			Vector2 center()const;
			void setWidth(const real& width);
			void setHeight(const real& height);
			void set(const real& width, const real& height);
		};

		Camera();
		void render(sf::RenderWindow& window);

		bool& aabbVisible();
		bool& jointVisible();
		bool& bodyVisible();
		bool& gridScaleLineVisible();
		bool& visible();
		bool& treeVisible();
		bool& centerVisible();
		bool& contactVisible();
		bool& uniformGridVisible();
		bool& contactImpulseVisible();
		bool& contactImpulseMagnitude();
		bool& contactFrictionVisible();
		bool& contactFrictionMagnitude();
		bool& bodyVelocity();
		bool& bodyVelocityNormal();
		bool& bodyVelocityMagnitude();
		bool& coordinateScale();
		bool& smoothZoom();
		bool& bodyIDVisible();
		bool& jointImpulseVisible();

		int axisPointCount() const;
		void setAxisPointCount(int count);

		real meterToPixel() const;
		void setTargetMeterToPixel(const real& meterToPixel);
		real pixelToMeter() const;

		Vector2 transform() const;
		void setTransform(const Vector2& transform);

		void setWorld(PhysicsWorld* world);
		PhysicsWorld* world() const;

		Body* targetBody() const;
		void setTargetBody(Body* targetBody);


		Viewport viewport() const;
		void setViewport(const Viewport& viewport);

		Vector2 worldToScreen(const Vector2& pos) const;
		Vector2 screenToWorld(const Vector2& pos) const;

		Tree* tree() const;
		void setTree(Tree* tree);


		real deltaTime() const;
		void setDeltaTime(const real& deltaTime);


		ContactMaintainer* maintainer() const;
		void setContactMaintainer(ContactMaintainer* maintainer);

		EasingType easingType() const;
		void setEasingType(EasingType type);


		UniformGrid* uniformGrid() const;
		void setUniformGrid(UniformGrid* grid);

		real defaultMeterToPixel()const;
		void setDefaultMeterToPixel(const real& number);

		void setPreScreenMousePos(const Vector2& pos);

		void setFont(sf::Font* font);
		sf::Font* font();
	private:
		void drawGridScaleLine(sf::RenderWindow& window);

		void drawTree(int nodeIndex, sf::RenderWindow& window);
		void drawContacts(sf::RenderWindow& window);

		bool m_visible = true;
		bool m_aabbVisible = false;
		bool m_jointVisible = true;
		bool m_bodyVisible = true;

		bool m_treeVisible = false;
		bool m_uniformGridVisible = false;
		bool m_gridScaleLineVisible = true;
		bool m_centerVisible = false;

		bool m_contactVisible = false;
		bool m_contactImpulseVisible = false;
		bool m_contactFrictionVisible = false;

		bool m_contactImpulseMagnitude = false;
		bool m_contactFrictionMagnitude = false;

		bool m_bodyVelocity = false;
		bool m_bodyVelocityNormal = false;
		bool m_bodyVelocityMagnitude = false;

		bool m_drawCoordinateScale = true;
		bool m_smoothZoom = true;

		bool m_bodyIDVisible = false;
		bool m_jointImpulseVisible = false;

		real m_defaultMeterToPixel = 50.0f;
		real m_meterToPixel = 50.0f;
		real m_pixelToMeter = 0.02f;

		real m_targetMeterToPixel = 80.0f;
		real m_targetPixelToMeter = 0.02f;

		Vector2 m_transform;
		Vector2 m_origin;
		Viewport m_viewport;
		PhysicsWorld* m_world = nullptr;
		Body* m_targetBody = nullptr;

		Tree* m_tree = nullptr;
		UniformGrid* m_grid = nullptr;
		ContactMaintainer* m_maintainer = nullptr;


		real m_restitution = 2.0f;
		real m_deltaTime = 15.0f;
		int m_axisPointCount = 50;

		Vector2 m_preScreenMousePos;
		Vector2 m_preWorldMousePos;

		EasingType m_easingType = EasingType::Exponential;

		sf::Font* m_font = nullptr;
	};
}
#endif
