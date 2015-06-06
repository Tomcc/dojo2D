#pragma once

#include "common_header.h"

#include "RayResult.h"
#include "ContactMode.h"
#include "WorldListener.h"

namespace Phys {
	class Body;

	class World :
		public b2ContactListener,
		public b2ContactFilter {
	public:
		typedef std::function<void()> Command;

		typedef std::unordered_set<const Body*> BodyList;
		typedef std::vector<const b2Fixture*> FixtureList;

		const float timeStep;

		static bool shapesOverlap(const b2Shape& s1, const b2Transform& t1, const b2Shape& s2, const b2Transform& t2);
		static bool shapesOverlap(const b2Shape& shape, const b2Fixture& fixture);

		World(const Vector& gravity, float timeStep, int velocityIterations, int positionIterations, int particleIterations);

		virtual ~World();

		void addListener(WorldListener& listener);
		void removeListener(WorldListener& listener);

		void setContactMode(Phys::Group A, Phys::Group B, ContactMode mode);
		ContactMode getContactModeFor(Phys::Group A, Phys::Group B) const;

		Vector getGravity() const;

		RayResult raycast(const Vector& start, const Vector& end, Phys::Group rayBelongsToGroup = 0) const;
		void asyncRaycast(const Vector& start, const Vector& end, Phys::Group rayBelongsToGroup, RayResult& result, const Command& callback = {}) const;
		bool _AABBQuery(const Vector& min, const Vector& max, Group group, BodyList* resultBody, FixtureList* resultFixture, bool precise = false) const;

		void AABBQuery(const Vector& min, const Vector& max, Group group, FixtureList& result, bool precise = false) const;
		void AABBQuery(const Vector& min, const Vector& max, Group group, BodyList& result, bool precise = false) const;

		bool AABBQueryEmpty(const Vector& min, const Vector& max, Group group, bool precise = false) const;

		void update(float dt);

		void _notifyDestroyed(Body& body);

		virtual void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) override;
		virtual bool ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB) override;
		virtual bool ShouldCollide(b2Fixture* fixture, b2ParticleSystem* particleSystem, int32 particleIndex) override;

		void sync() const;

		void asyncCommand(Command command, const Command& callback = {}) const;
		void asyncCallback(const Command& callback) const;
		bool isWorkerThread() const;

		b2World& getBox2D() {
			return *box2D;
		}
		void addBody(Body& body);
		void removeBody(Body& body);

		void pause();

		void resume();

		bool isPaused() const {
			return simulationPaused;
		}

		void startPhysics() {
			resume();
		}

	protected:

		struct DeferredCollision {
			Body* A, *B;
			float force;
			b2Vec2 point;
			DeferredCollision() {}
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
			DeferredSensorCollision() {}
			DeferredSensorCollision(Body& other, Body& me, b2Fixture& sensor) :
				other(&other),
				me(&me),
				sensor(&sensor) {

			}
		};

		struct Job {
			Command command, callback;
			Job() {}
			Job(const Command& command, const Command& callback) :
				command(command),
				callback(callback) {

			}
		};

		std::thread thread;

		bool running = true;
			
		Dojo::SmallSet<WorldListener*> listeners;

		Unique<b2World> box2D;

		Unique<Dojo::Pipe<Job>> commands;
		Unique<Dojo::Pipe<Command>> callbacks;
		Unique<Dojo::Pipe<DeferredCollision>> deferredCollisions;
		Unique<Dojo::Pipe<DeferredSensorCollision>> deferredSensorCollisions;

		Dojo::SmallSet<Body*> bodies, deletedBodies;

		static const int GROUP_COUNT = 256; //HACK
		ContactMode collideMode[GROUP_COUNT][GROUP_COUNT];

		bool simulationPaused = true;
	};
}



