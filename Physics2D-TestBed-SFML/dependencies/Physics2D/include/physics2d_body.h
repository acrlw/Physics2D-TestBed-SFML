#ifndef PHYSICS2D_BODY_H
#define PHYSICS2D_BODY_H
#include "physics2d_aabb.h"
#include "physics2d_math.h"
#include "physics2d_common.h"
#include "physics2d_shape.h"
#include "physics2d_integrator.h"

#include "physics2d_shape_capsule.h"
#include "physics2d_shape_circle.h"
#include "physics2d_shape_curve.h"
#include "physics2d_shape_edge.h"
#include "physics2d_shape_ellipse.h"
#include "physics2d_shape_point.h"
#include "physics2d_shape_polygon.h"
#include "physics2d_shape_rectangle.h"
#include "physics2d_shape_sector.h"

namespace Physics2D
{
	enum class BodyType
	{
		Kinematic,
		Static,
		Dynamic,
		Bullet
	};
	struct Relation
	{
		using RelationID = uint64_t;
		static RelationID generateRelationID(Body* bodyA, Body* bodyB);
		static Relation generateRelation(Body* bodyA, Body* bodyB);
		RelationID relationID;
		Body* bodyA;
		Body* bodyB;
	};
	struct PhysicsTransform
	{
		PhysicsTransform() = default;
		PhysicsTransform(const Vec2& pos, const Vec2& v, const real& rotate, const real& omega);
		Vec2 position;
		Vec2 velocity;
		real rotation = 0;
		real angularVelocity = 0;
		void step(const real& dt);
	};
	class Body
	{
	public:

		Body() = default;
		Vec2& position();

		Vec2& velocity();

		real& rotation();

		real& angularVelocity();

		Vec2& forces();
		void clearTorque();

		real& torques();

		Vec2& lastPosition();
		real& lastRotation();
		uint32_t& sleepCountdown();

		Shape* shape()const;
		void setShape(Shape* shape);

		BodyType type() const;
		void setType(const BodyType& type);

		real mass() const;
		void setMass(const real& mass);

		real inertia() const;

		AABB aabb(const real& factor = Constant::AABBExpansionFactor) const;

		real friction() const;
		void setFriction(const real& friction);

		bool sleep() const;
		void setSleep(bool sleep);

		real inverseMass() const;
		real inverseInertia() const;

		PhysicsTransform physicsTransform() const;
		void setPhysicsTransform(const PhysicsTransform& info);

		void stepPosition(const real& dt);

		void applyImpulse(const Vec2& impulse, const Vec2& r);
		Vec2 toLocalPoint(const Vec2& point) const;
		Vec2 toWorldPoint(const Vec2& point) const;
		Vec2 toActualPoint(const Vec2& point) const;

		uint32_t id()const;
		void setId(const uint32_t& id);

		uint32_t bitmask()const;
		void setBitmask(const uint32_t& bitmask);

		real restitution()const;
		void setRestitution(const real& restitution);


	private:
		void calcInertia();

		uint32_t m_id;
		uint32_t m_bitmask = 1;

		real m_mass = 0;
		real m_inertia = 0;
		real m_invMass = 0;
		real m_invInertia = 0;

		Vec2 m_position;
		Vec2 m_velocity;
		real m_rotation = 0;
		real m_angularVelocity = 0;

		Vec2 m_lastPosition;
		real m_lastRotation = 0;

		Vec2 m_forces;
		real m_torques = 0;

		Shape* m_shape;
		BodyType m_type = BodyType::Static;

		bool m_sleep = false;
		real m_friction = 0.1f;
		real m_restitution = 0.0f;
		
		uint32_t m_sleepCountdown = 0;

		
	};
}
#endif
