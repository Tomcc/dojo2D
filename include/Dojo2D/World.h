#pragma once

#include "common_header.h"

#include "RayResult.h"

namespace Phys {
	class Body;

	class World :
		public b2ContactListener,
		public b2ContactFilter {
	public:
		typedef std::unordered_set<const Body*> BodyList;
		typedef std::vector<const b2Fixture*> FixtureList;

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

		static bool shapesOverlap(const b2Shape& s1, const b2Transform& t1, const b2Shape& s2, const b2Transform& t2);
		static bool shapesOverlap(const b2Shape& shape, const b2Fixture& fixture);

		World(const Vector& gravity, float timeStep, int velocityIterations, int positionIterations, int particleIterations);

		void setContactMode(Phys::Group A, Phys::Group B, ContactMode mode);
		ContactMode getContactModeFor(Phys::Group A, Phys::Group B) const;

		b2World& getBox2D() {
			return *box2D;
		}

		Vector getGravity() const;

		RayResult raycast(const Vector& start, const Vector& end, Phys::Group rayBelongsToGroup = 0) const;

		bool _AABBQuery(const Vector& min, const Vector& max, Group group, BodyList* resultBody, FixtureList* resultFixture, bool precise = false) const;

		void AABBQuery(const Vector& min, const Vector& max, Group group, FixtureList& result, bool precise = false);
		void AABBQuery(const Vector& min, const Vector& max, Group group, BodyList& result, bool precise = false);

		bool AABBQueryEmpty(const Vector& min, const Vector& max, Group group, bool precise = false) const;

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
			Body* other, *me;
			b2Fixture* sensor;
			DeferredSensorCollision(Body& other, Body& me, b2Fixture& sensor) :
				other(&other),
				me(&me),
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



