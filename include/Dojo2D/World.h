#pragma once

#include "common_header.h"

#include "RayResult.h"
#include "ContactMode.h"
#include "WorldListener.h"
#include "AABBQueryResult.h"

namespace Phys {
	class Body;
	class BodyPart;

	class World :
		public b2ContactListener,
		public b2ContactFilter {
	public:
		typedef std::function<void()> Command;
		typedef std::function<float()> Controller;

	protected:
		struct DeferredCollision {
			std::weak_ptr<BodyPart> A, B;
			float force;
			b2Vec2 point;
			DeferredCollision() {}
			DeferredCollision(std::weak_ptr<BodyPart> A, std::weak_ptr<BodyPart> B, float force, const b2Vec2& point) :
				A(A),
				B(B),
				force(force),
				point(point) {

			}
		};

		struct DeferredSensorCollision {
			Body* other, *me;
			std::weak_ptr<BodyPart> sensor;
			DeferredSensorCollision() {}
			DeferredSensorCollision(Body& other, Body& me, std::weak_ptr<BodyPart> sensor) :
				other(&other),
				me(&me),
				sensor(sensor) {

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

		enum AABBQueryFlags {
			QUERY_BODIES = 1 << 1,
			QUERY_FIXTURES = 1 << 2,
			QUERY_PARTICLES = 1 << 3,
			QUERY_PRECISE = 1 << 4,
			QUERY_PUSHABLE_ONLY = 1 << 5
		};

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

		void playCollisionSound(const DeferredCollision& collision, const BodyPart& part);
		std::future<RayResult> raycast(const Vector& start, const Vector& end, Phys::Group rayBelongsToGroup = 0) const;
		std::future<AABBQueryResult> AABBQuery(const Dojo::AABB& area, Group group, uint8_t flags = 0) const;

		void applyForceField(const Dojo::AABB& area, Group group, const Vector& force, FieldType type);

		void update(float dt);

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

		Dojo::SmallSet<Body*> bodies;

		static const int GROUP_COUNT = 256; //HACK
		ContactMode collideMode[GROUP_COUNT][GROUP_COUNT];
		
		float removeNextSound = 0;
		std::deque<Vector> recentlyPlayedSoundPositions;

		bool simulationPaused = true;

		float _closestRecentlyPlayedSound(const Vector& point);
	};
}



