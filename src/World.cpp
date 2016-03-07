#include "World.h"
#include "Body.h"
#include "BodyPart.h"
#include "PhysUtil.h"
#include "ParticleSystem.h"
#include "Material.h"

using namespace Phys;

const static b2Transform B2_IDENTITY = {b2Vec2(0.f, 0.f), b2Rot(0.f)};

bool World::shapesOverlap(const b2Shape& s1, const b2Transform& t1, const b2Shape& s2, const b2Transform& t2) {
	return b2TestOverlap(&s1, 0, &s2, 0, t1, t2);
}

bool World::shapesOverlap(const b2Shape& shape, const b2Fixture& fixture) {
	return shapesOverlap(shape, B2_IDENTITY, *fixture.GetShape(), fixture.GetBody()->GetTransform());
}


World::World(const Vector& gravity, float timeStep, int velocityIterations, int positionIterations, int particleIterations) :
	timeStep(timeStep) {

	DEBUG_ASSERT(timeStep > 0, "Invalid timestep");

	for (int i = 0; i < GROUP_COUNT; ++i) {
		for (int j = 0; j < GROUP_COUNT; ++j) {
			collideMode[j][i] = ContactMode::Normal;
		}
	}

	//create the box 2D world
	box2D = make_unique<b2World>(asB2Vec(gravity));
	box2D->SetContactListener(this);
	box2D->SetContactFilter(this);

	commands = make_unique<Dojo::Pipe<Job>>();
	callbacks = make_unique<Dojo::Pipe<Command>>();
	deferredCollisions = make_unique<Dojo::Pipe<DeferredCollision>>();
	deferredSensorCollisions = make_unique<Dojo::Pipe<DeferredSensorCollision>>();

	thread = std::thread([ = ]() {
		Dojo::Timer timer;
		Job job;

		while (running) {

			//process all available commands
			while ((simulationPaused || timer.getElapsedTime() < timeStep) && commands->try_dequeue(job)) {
				job.command();

				if (job.callback) {
					callbacks->enqueue(std::move(job.callback));
				}
			}

			if (!simulationPaused && timer.getElapsedTime() >= timeStep) {
				timer.reset();
				box2D->Step(timeStep, velocityIterations, positionIterations, particleIterations);

				for (auto&& b : bodies) {
					if (b->getB2Body()->IsAwake() && b->getB2Body()->IsActive()) {
						b->updateObject();
					}
				}

				for (auto&& listener : listeners) {
					listener->onPostSimulationStep();
				}
			}
			else {
				std::this_thread::yield();
			}
		}
	});
}

World::~World() {
	running = false;
	thread.join();
}

void World::addListener(WorldListener& listener) {
	asyncCommand([&] {
		listeners.emplace(&listener);
	});
}

void World::removeListener(WorldListener& listener) {
	asyncCommand([&] {
		listeners.erase(&listener);
	});
}

void World::setContactMode(Group A, Group B, ContactMode mode) {
	collideMode[A][B] = collideMode[B][A] = mode;
}

ContactMode World::getContactModeFor(Group A, Group B) const {
	auto modeA = collideMode[A][B];
	auto modeB = collideMode[B][A];

	return std::min(modeA, modeB);
}

void World::asyncCommand(Command command, const Command& callback /*= Command()*/) const {
	DEBUG_ASSERT(command, "Command can't be a NOP");

	if (isWorkerThread()) {
		command();

		if (callback) {
			callbacks->enqueue(callback);
		}
	}
	else {
		commands->enqueue(std::move(command), callback);
	}
}

void World::asyncCallback(const Command& callback) const {
	DEBUG_ASSERT(callback, "Command can't be a NOP");

	if (isWorkerThread()) {
		callbacks->enqueue(callback);
	}
	else {
		callback();
	}
}

void World::sync() const {
	if (!isWorkerThread()) {
		std::atomic<bool> done = false;
		asyncCommand([&] {
			done = true;
		});

		while (!done) {
			std::this_thread::yield();
		}
	}
}

bool World::ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB) {

	auto& phA = getBodyForFixture(fixtureA);
	auto& phB = getBodyForFixture(fixtureB);

	auto cm = getContactModeFor(phA.getGroup(), phB.getGroup());

	if (cm != ContactMode::Normal) {
		//a "ghost collision" acts like a two-way sensor
		if (cm == ContactMode::Ghost && !fixtureA->IsSensor() && !fixtureB->IsSensor()) {
			deferredSensorCollisions->enqueue(phB, phA, *fixtureA);
			deferredSensorCollisions->enqueue(phA, phB, *fixtureB);
		}

		return false;
	}

	//check if the sensors should collide
	if (fixtureA->IsSensor()) {
		deferredSensorCollisions->enqueue(phB, phA, *fixtureA);
	}

	if (fixtureB->IsSensor()) {
		deferredSensorCollisions->enqueue(phA, phB, *fixtureB);
	}

	if (fixtureA->IsSensor() && fixtureB->IsSensor()) {
		return false;
	}

	bool odcA = phA.isParticle();
	bool odcB = phB.isParticle();

	if (odcA != odcB) { //only one!

		auto shouldBeBelow = odcA ? fixtureB : fixtureA;
		auto shouldBeAbove = odcA ? fixtureA : fixtureB;

		//check if they are in the correct position
		if (shouldBeBelow->GetBody()->GetPosition().y > shouldBeAbove->GetBody()->GetPosition().y - shouldBeAbove->GetAABB(0).GetExtents().y * 0.3f) {
			return false;
		}
	}

	return b2ContactFilter::ShouldCollide(fixtureA, fixtureB);
}

bool World::ShouldCollide(b2Fixture* fixture, b2ParticleSystem* particleSystem, int32 particleIndex) {
	auto& body = getBodyForFixture(fixture);
	auto& ps = ParticleSystem::getFor(particleSystem);

	if (getContactModeFor(body.getGroup(), ps.group) != ContactMode::Normal) {
		return false;
	}
	else {
		return b2ContactFilter::ShouldCollide(fixture, particleSystem, particleIndex);
	}
}

void World::PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) {
	b2WorldManifold worldManifold;

	auto& partA = getPartForFixture(contact->GetFixtureA());
	auto& partB = getPartForFixture(contact->GetFixtureB());
	auto& bodyA = partA.body;
	auto& bodyB = partB.body;
	DEBUG_ASSERT(bodyA.isStatic() || bodyA.getMass() > 0, "HM");
	DEBUG_ASSERT(bodyB.isStatic() || bodyB.getMass() > 0, "HM");

	//don't report collisions between bodies with no listeners, duh
	if (!bodyA.collisionListener && !bodyB.collisionListener) {
		return;
	}

	contact->GetWorldManifold(&worldManifold);

	b2Vec2 point;

	if (contact->GetManifold()->pointCount == 1) {
		point = worldManifold.points[0];
	}
	else {
		point = (worldManifold.points[0] + worldManifold.points[1]) * 0.5f;
	}


	auto& N = worldManifold.normal;
	//	b2Vec2 T = { N.y, -N.x };
	b2Vec2 F = {0, 0};

	for (int i = 0; i < impulse->count; ++i) {
		F += impulse->normalImpulses[i] * N;
	}

	float force = F.Length();

	if (force > 0.1f) {
		deferredCollisions->enqueue(partA, partB, force, point);
	}
}

std::future<RayResult> World::raycast(const Vector& start, const Vector& end, Group rayBelongsToGroup) const {
	auto promise = make_shared<std::promise<RayResult>>(); //TODO pool the promises perhaps

	asyncCommand([this, promise, start, end, rayBelongsToGroup]() {
		RayResult result(self);
		result.group = rayBelongsToGroup;

		box2D->RayCast(
			&result,
		{ start.x, start.y },
		{ end.x, end.y });

		if (!result.hit) {
			result.position = end;
		}

		result.dist = start.distance(result.position);

		promise->set_value(result);
	});

	return promise->get_future();
}

bool World::isWorkerThread() const {
	return std::this_thread::get_id() == thread.get_id();
}

std::future<AABBQueryResult> World::AABBQuery(const Dojo::AABB& area, Group group, uint8_t flags) const {
	auto promise = make_shared<std::promise<AABBQueryResult>>(); //TODO pool results
	asyncCommand([this, promise, flags, area, group] {
		b2PolygonShape aabbShape;
		AABBQueryResult result;
		
		if (flags & QUERY_PRECISE) {
			Vector dim = area.getSize();
			aabbShape.SetAsBox(dim.x, dim.y, asB2Vec(area.getCenter()), 0);
		}

		auto report = [&](b2Fixture * fixture) {
			if (!fixture->IsSensor()) {
				auto& body = getBodyForFixture(fixture);
				if (!(flags & QUERY_PUSHABLE_ONLY) || body.isPushable()) {
					auto contactMode = getContactModeFor(group, body.getGroup());

					if (contactMode == ContactMode::Normal) {

						if (!(flags & QUERY_PRECISE) || shapesOverlap(aabbShape, *fixture)) {
							result.empty = false;

							if (flags & QUERY_BODIES) {
								result.bodies.insert(&body);
							}

							else if (flags & QUERY_FIXTURES) {
								result.fixtures.emplace_back(fixture);
							}
							else {
								return false;    //stop search immediately
							}
						}
					}
				}
			}

			return true;
		};

		class Query : public b2QueryCallback {
		public:
			decltype(report)& func;
			AABBQueryResult::ParticleList& particles;
			bool queryParticles;

			Query(const decltype(func)& f, AABBQueryResult::ParticleList& p, bool queryParticles) 
				: func(f)
				, particles(p) {
			}

			virtual bool ReportFixture(b2Fixture* fixture) override {
				return func(fixture);
			}

			virtual bool ReportParticle(b2ParticleSystem* particleSystem, int32 index) override {
				//TODO //WARNING LiquidFun's particle reporting mechanics look like really inefficient
				//jumbles of virtuals, this can probably be optimized
				
				particles[particleSystem].emplace_back(index);
				return true;
			}

			virtual bool ShouldQueryParticleSystem(const b2ParticleSystem* particleSystem) override {
				return queryParticles;
			}
		};

		b2AABB bb;
		bb.lowerBound = asB2Vec(area.min);
		bb.upperBound = asB2Vec(area.max);

		Query q = { report, result.particles, (flags & QUERY_PARTICLES) > 0 };
		box2D->QueryAABB(&q, bb);

		promise->set_value(std::move(result));
	});
	
	return promise->get_future();
}

void Phys::World::applyForceField(const Dojo::AABB& area, Group group, const Vector& force, FieldType type) {
	asyncCommand([this, area, group, force] {
		auto F = asB2Vec(force);
		auto query = AABBQuery(area, group, QUERY_FIXTURES | QUERY_PARTICLES | QUERY_PUSHABLE_ONLY);

		for (auto&& fixture : query.get().fixtures) {
			//TODO the force should be proportional to the area or to the volume
			fixture->GetBody()->ApplyForceToCenter(F, true);
		}

		for (auto&& pair : query.get().particles) {
			for (auto&& i : pair.second) {
				pair.first->ParticleApplyForce(i, F);
			}
		}
	});
}


Vector World::getGravity() const {
	return asVec(box2D->GetGravity());
}

const float MIN_SOUND_FORCE = 1.f;

float Phys::World::_closestRecentlyPlayedSound(const Vector& point) {
	float minDist = FLT_MAX;
	for (auto&& pos : recentlyPlayedSoundPositions) {
		minDist = std::min(minDist, pos.distanceSquared(point));
	}
	return sqrt(minDist);
}

void Phys::World::playCollisionSound(const DeferredCollision& collision) {
	//TODO choose which sound to play... both? random? existing?
	//TODO this code doesn't belong here too much, perhaps World shouldn't know about sounds
	auto& part = *collision.A;

	if (collision.force > MIN_SOUND_FORCE) {
		//ensure that the bodies are actually moving respect to each other
		auto& impactSound = (collision.force > 5) ? part.material.impactHard : part.material.impactSoft;
		if (auto set = impactSound.to_ref()) {

			auto pos = asVec(collision.point);
			if (_closestRecentlyPlayedSound(pos) > 0.2f) {
				float volume = std::min(collision.force / 3.f, 1.f);
				Dojo::Platform::singleton().getSoundManager().playSound(
					pos,
					set.get(),
					volume
				);

				if (recentlyPlayedSoundPositions.size() > 20) {
					recentlyPlayedSoundPositions.pop_front();
				}
				recentlyPlayedSoundPositions.emplace_back(pos);
			}
		}
	}
}



void World::update(float dt) {
	DEBUG_ASSERT(!isWorkerThread(), "Wrong Thread");

	//remove a recently played sound
	if (recentlyPlayedSoundPositions.size() > 0) {
		removeNextSound += dt;
		if (removeNextSound > 0.5f) {
			recentlyPlayedSoundPositions.pop_front();
			removeNextSound = 0;
		}
	}

	//play back all collisions
	DeferredCollision c;

	while (deferredCollisions->try_dequeue(c)) {
		auto& bA = c.A->body;
		auto& bB = c.B->body;

		if (deletedBodies.contains(&bA) || deletedBodies.contains(&bB)) {
			continue;
		}

		Vector p = asVec(c.point);

		if (bA.collisionListener) {
			bA.collisionListener->onCollision(bA, bB, c.force, p);
		}

		if (bB.collisionListener) {
			bB.collisionListener->onCollision(bB, bA, c.force, p);
		}

		playCollisionSound(c);
	}

	DeferredSensorCollision sc;

	while (deferredSensorCollisions->try_dequeue(sc)) {
		auto& body = getBodyForFixture(sc.sensor);

		if (deletedBodies.contains(&body)) {
			continue;
		}

		if (body.collisionListener) {
			body.collisionListener->onSensorCollision(*sc.other, *sc.sensor);    //sensor collisions are not bidirectional
		}
	}

	//TODO timeslice this in some way?
	//right now it can stall the main thread with too many callbacks
	Command callback;

	while (callbacks->try_dequeue(callback)) {
		callback();
	}

	//by now, any stale pointer in the queues should have been cleaned
	deletedBodies.clear();
}

void World::_notifyDestroyed(Body& body) {
	DEBUG_ASSERT(!isWorkerThread(), "Wrong Thread");
	deletedBodies.emplace(&body);
}

void World::addBody(Body& body) {
	DEBUG_ASSERT(isWorkerThread(), "Wrong Thread");
	bodies.emplace(&body);
}

void World::removeBody(Body& body) {
	DEBUG_ASSERT(isWorkerThread(), "Wrong Thread");
	bodies.erase(&body);
}

void World::pause() {
	asyncCommand([this]() {
		DEBUG_ASSERT(!simulationPaused, "Already paused");
		simulationPaused = true;

		//tell all bodies they're being paused
		for (auto&& body : bodies) {
			body->onSimulationPaused();
		}
	});
}

void World::resume() {
	asyncCommand([this]() {
		DEBUG_ASSERT(simulationPaused, "Already resumed");
		simulationPaused = false;
	});
}
