#pragma once

#include "common_header.h"

#include "RayResult.h"

namespace Phys {
	class Body;

	class World :
		public b2ContactListener,
		public b2ContactFilter {
	public:
		typedef std::vector<Body*> BodyList;

		enum class ContactMode {
			None,
			Normal,
			Ghost
		};

		const float timeStep;

		const int32 
			velocityIterations,
			positionIterations,
			particleIterations;

		World(const Vector& gravity, float timeStep, int velocityIterations, int positionIterations, int particleIterations);

		void setContactMode(Phys::Group A, Phys::Group B, ContactMode mode);
		ContactMode getContactModeFor(Phys::Group A, Phys::Group B) const;

		b2World& getBox2D() {
			return *box2D;
		}

		Vector getGravity() const;

		RayResult raycast(const Vector& start, const Vector& end, Phys::Group rayBelongsToGroup = 0);

		void AABBQuery(const Vector& min, const Vector& max, Group group, BodyList& result);
		void AABBQuery(const Dojo::Object& bounds, Group group, BodyList& result);

		void update(float dt);

		void _notifyDestroyed(Body& body);

		virtual void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse);
		virtual bool ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB);

	protected:

		struct DeferredCollision {
			Body* A, *B;
			float force;
			b2Vec2 point;

			DeferredCollision(Body& A, Body& B, float force, const b2Vec2& point) :
				A(&A),
				B(&B),
				force(force),
				point(point) {

			}
		};

		struct DeferredSensorCollision {
			Body* other;
			b2Fixture* sensor;
			DeferredSensorCollision(Body& other, b2Fixture& sensor) :
				other(&other),
				sensor(&sensor) {

			}
		};
		
		Unique<b2World> box2D;

		std::vector<DeferredCollision> deferredCollisions;
		std::vector<DeferredSensorCollision> deferredSensorCollisions;

		static const int GROUP_COUNT = 256; //HACK
		ContactMode collideMode[GROUP_COUNT][GROUP_COUNT];
	};
}



