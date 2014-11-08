#pragma once

#include "common_header.h"

#include "RayResult.h"

namespace Phys {
	class World :
		public b2ContactListener,
		public b2ContactFilter {
	public:
		typedef std::vector<b2Fixture*> FixtureList;

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

		void AABBQuery(const Vector& min, const Vector& max, FixtureList& result);
		void AABBQuery(const Dojo::Object& bounds, FixtureList& result);

		void update(float dt);

		virtual void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse);
		virtual bool ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB);

	protected:

		struct DeferredCollision {
			Phys::Body& A, &B;
			float force;
			b2Vec2 point;

			DeferredCollision(Phys::Body& A, Phys::Body& B, float force, const b2Vec2& point) :
				A(A),
				B(B),
				force(force),
				point(point) {

			}
		};

		Unique<b2World> box2D;

		std::vector<DeferredCollision> deferredCollisions, deferredGhostCollisions;

		static const int GROUP_COUNT = 256; //HACK
		ContactMode collideMode[GROUP_COUNT][GROUP_COUNT];
	};
}



