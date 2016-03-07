#pragma once

#include "common_header.h"

#include "RayResult.h"
#include "ContactMode.h"
#include "WorldListener.h"

namespace Phys {
	class Body;
	class BodyPart;

	class World :
		public b2ContactListener,
		public b2ContactFilter {
	public:
		typedef std::function<void()> Command;

	protected:
		struct DeferredCollision {
			BodyPart* A, *B;
			float force;
			b2Vec2 point;
			DeferredCollision() {}
			DeferredCollision(BodyPart& A, BodyPart& B, float force, const b2Vec2& point) :
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

	public:

		enum class FieldType {
			ApplyToSurface,
			ApplyToVolume
		};

		typedef std::unordered_set<Body*> BodyList;
		typedef std::vector<b2Fixture*> FixtureList;
		typedef std::unordered_map<b2ParticleSystem*, std::vector<uint16_t>> ParticleList;

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

		void playCollisionSound(const DeferredCollision& collision);
		std::future<RayResult> raycast(const Vector& start, const Vector& end, Phys::Group rayBelongsToGroup = 0) const;
		bool _AABBQuery(const Dojo::AABB& area, Group group, BodyList* resultBody, FixtureList* resultFixture, ParticleList* particles, bool precise, bool onlyPushable) const;

		void AABBQuery(const Dojo::AABB& area, Group group, FixtureList& result, bool precise = false, ParticleList* particles = nullptr) const;
		void AABBQuery(const Dojo::AABB& area, Group group, BodyList& result, bool precise = false, ParticleList* particles = nullptr) const;

		bool AABBQueryEmpty(const Dojo::AABB& area, Group group, bool precise = false) const;

		void applyForceField(const Dojo::AABB& area, Group group, const Vector& force, FieldType type);

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

		const b2World& getBox2D() const {
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
		
		float removeNextSound = 0;
		std::deque<Vector> recentlyPlayedSoundPositions;

		bool simulationPaused = true;

		float _closestRecentlyPlayedSound(const Vector& point);
	};
}



