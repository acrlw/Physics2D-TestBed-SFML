#ifndef PHYSICS2D_WORLD_H
#define PHYSICS2D_WORLD_H
#include "../common/common.h"
#include "../dynamics/body.h"
#include "../math/math.h"
#include "../math/integrator.h"
#include "../dynamics/joints/joints.h"
#include "../utils/random.h"
#include "../dynamics/contact/contact.h"
namespace Physics2D
{
    class PhysicsWorld
    {
		public:
            PhysicsWorld() : m_gravity(0, -9.8f), m_linearVelocityDamping(0.9f), m_angularVelocityDamping(0.9f),
                             m_linearVelocityThreshold(0.02f),
                             m_angularVelocityThreshold(0.02f), m_airFrictionCoefficient(0.7f), m_bias(0.8f)
            {}
            ~PhysicsWorld();
            void prepareVelocityConstraint(const real& dt);
            void stepVelocity(const real& dt);
            void solveVelocityConstraint(real dt);
            void stepPosition(const real& dt);
            void solvePositionConstraint(real dt);
            

            Vec2 gravity() const;
            void setGravity(const Vec2 &gravity);

            real linearVelocityDamping() const;
            void setLinearVelocityDamping(const real&linearVelocityDamping);

            real angularVelocityDamping() const;
            void setAngularVelocityDamping(const real &angularVelocityDamping);

            real linearVelocityThreshold() const;
            void setLinearVelocityThreshold(const real&linearVelocityThreshold);

            real angularVelocityThreshold() const;
            void setAngularVelocityThreshold(const real &angularVelocityThreshold);

            real airFrictionCoefficient()const;
    		void setAirFrictionCoefficient(const real& airFrictionCoefficient);

            bool enableGravity() const;
            void setEnableGravity(bool enableGravity);

            bool enableDamping() const;
            void setEnableDamping(bool enableDamping);
            
            Body* createBody();
            void removeBody(Body* body);

            void removeJoint(Joint* joint);

            void clearAllBodies();
            void clearAllJoints();
    	
            RotationJoint* createJoint(const RotationJointPrimitive& primitive);
            PointJoint* createJoint(const PointJointPrimitive& primitive);
            DistanceJoint* createJoint(const DistanceJointPrimitive& primitive);
            PulleyJoint* createJoint(const PulleyJointPrimitive& primitive);
            RevoluteJoint* createJoint(const RevoluteJointPrimitive& primitive);
            OrientationJoint* createJoint(const OrientationJointPrimitive& primitive);
            
            real bias() const;
            void setBias(const real &bias);

            std::vector<std::unique_ptr<Body>>& bodyList();
    	
            std::vector<std::unique_ptr<Joint>>& jointList();
        private:

            Vec2 m_gravity;
            real m_linearVelocityDamping;
            real m_angularVelocityDamping;
            real m_linearVelocityThreshold;
            real m_angularVelocityThreshold;
            real m_airFrictionCoefficient;

            real m_bias;
    		
    		bool m_enableGravity = true;
    		bool m_enableDamping = true;
            std::vector<std::unique_ptr<Body>> m_bodyList;
            std::vector<std::unique_ptr<Joint>> m_jointList;
    		
    };
    using Index = size_t;
    class DiscretePhysicsWorld
    {
    public:

        Index createBody();
        Index createJoint();

        Vec2 gravity()const;
        void setGravity(const Vec2& gravity);

        Vec2 linearDamping()const;
        void setLinearDamping(real damping);

        Vec2 angularDamping()const;
        void setAngularDamping(real damping);

        void prepareVelocityVelocity(real dt);
        void solveVelocityConstraint(real dt);
        void solvePositionConstraint(real dt);
        void stepVelocity(real dt);
        void stepPosition(real dt);
        void stepSleep();
    private:
        //Index relates to body
    	std::vector<Vec2> position;
        std::vector<Vec2> velocity;

        std::vector<real> rotation;
        std::vector<real> angularVelocity;

        std::vector<Vec2> lastPosition;
        std::vector<Vec2> lastRotation;

        std::vector<Vec2> force;
        std::vector<real> torque;

        std::vector<real> invMass;
        std::vector<real> invInertia;

        std::vector<real> friction;
        std::vector<real> restitution;

        std::vector<Shape*> shape;
        std::vector<uint8_t> sleep;//0: awake, 1: sleep
        std::vector<BodyType> type;
        std::vector<uint32_t> bitmask;

        std::vector<uint32_t> sleepTimer;
        std::vector<real> energy;

        //Index independent
        std::vector<std::unique_ptr<Shape*>> shapePool;
        std::vector<size_t> freeList;
        
        Vec2 m_gravity = Vec2(0, -9.8f);
        real m_linearDamping = 1.0f;
        real m_angularDamping = 1.0f;
    };
}
#endif
