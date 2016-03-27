#include "World.h"
#include "Body.h"
#include "BodyPart.h"
#include "PhysUtil.h"
#include "ParticleSystem.h"
#include "Material.h"
#include "Joint.h"

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
			mCollideMode[j][i] = ContactMode::Normal;
		}
	}

	//create the box 2D world
	mBox2D = make_unique<b2World>(asB2Vec(gravity));
	mBox2D->SetContactListener(this);
	mBox2D->SetContactFilter(this);

	mCommands = make_unique<Dojo::SPSCQueue<Job>>();
	mCallbacks = make_unique<Dojo::SPSCQueue<Command>>();
	mDeferredCollisions = make_unique<Dojo::SPSCQueue<DeferredCollision>>();
	mDeferredSensorCollisions = make_unique<Dojo::SPSCQueue<DeferredSensorCollision>>();

	mThread = std::thread([ = ]() {
		Dojo::Timer timer;
		Job job;

		while (mRunning) {

			//process all available commands
			while ((mSimulationPaused || timer.getElapsedTime() < timeStep) && mCommands->try_dequeue(job)) {
				job.command();

				if (job.callback) {
					mCallbacks->enqueue(std::move(job.callback));
				}
			}

			if (!mSimulationPaused && timer.getElapsedTime() >= timeStep) {
				auto step = static_cast<float>(timer.getElapsedTime());
				timer.reset();
				mBox2D->Step(step, velocityIterations, positionIterations, particleIterations);

				for (auto&& b : mBodies) {
					auto& body = b->getB2Body().unwrap();
					if (body.IsAwake() && body.IsActive()) {
						b->updateObject();
					}
				}

				for (auto&& listener : mListeners) {
					listener->onPhysicsStep(step);
				}
			}
			else {
				std::this_thread::yield();
			}
		}
	});
}

World::~World() {
	mRunning = false;
	mThread.join();
}

void World::addListener(WorldListener& listener) {
	asyncCommand([&] {
		mListeners.emplace(&listener);
	});
}

void World::removeListener(WorldListener& listener) {
	asyncCommand([&] {
		mListeners.erase(&listener);
	});
}

void World::setContactMode(Group A, Group B, ContactMode mode) {
	mCollideMode[A][B] = mCollideMode[B][A] = mode;
}

ContactMode World::getContactModeFor(Group A, Group B) const {
	auto modeA = mCollideMode[A][B];
	auto modeB = mCollideMode[B][A];

	return std::min(modeA, modeB);
}

void World::asyncCommand(Command command, const Command& callback /*= Command()*/) const {
	DEBUG_ASSERT(command, "Command can't be a NOP");

	if (isWorkerThread()) {
		command();

		if (callback) {
			mCallbacks->enqueue(callback);
		}
	}
	else {
		mCommands->enqueue(std::move(command), callback);
	}
}

void World::asyncCallback(const Command& callback) const {
	DEBUG_ASSERT(callback, "Command can't be a NOP");

	if (isWorkerThread()) {
		mCallbacks->enqueue(callback);
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

	auto& partA = getPartForFixture(fixtureA);
	auto& partB = getPartForFixture(fixtureB);

	auto cm = getContactModeFor(partA.body.getGroup(), partB.body.getGroup());

	if (cm != ContactMode::Normal) {
		//a "ghost collision" acts like a two-way sensor
		if (cm == ContactMode::Ghost && !fixtureA->IsSensor() && !fixtureB->IsSensor()) {
			mDeferredSensorCollisions->enqueue(partB.body, partA.body, partA._getWeakPtr());
			mDeferredSensorCollisions->enqueue(partA.body, partB.body, partB._getWeakPtr());
		}

		return false;
	}

	//check if the sensors should collide
	if (fixtureA->IsSensor()) {
		mDeferredSensorCollisions->enqueue(partB.body, partA.body, partA._getWeakPtr());
	}

	if (fixtureB->IsSensor()) {
		mDeferredSensorCollisions->enqueue(partA.body, partB.body, partB._getWeakPtr());
	}

	if (fixtureA->IsSensor() && fixtureB->IsSensor()) {
		return false;
	}

	bool odcA = partA.body.isParticle();
	bool odcB = partB.body.isParticle();

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
		mDeferredCollisions->enqueue(partA._getWeakPtr(), partB._getWeakPtr(), force, point);
	}
}

std::future<RayResult> World::raycast(const Vector& start, const Vector& end, Group rayBelongsToGroup) const {
	auto promise = make_shared<std::promise<RayResult>>(); //TODO pool the promises perhaps

	DEBUG_ASSERT(end.isValid(), "HHMM");

	asyncCommand([this, promise, start, end, rayBelongsToGroup]() {
		RayResult result(self);
		result.group = rayBelongsToGroup;

		mBox2D->RayCast(
			&result,
			{ start.x, start.y },
			{ end.x, end.y }
		);

		if (!result.hit) {
			result.position = end;
		}


		result.dist = start.distance(result.position);

		promise->set_value(result);
	});

	return promise->get_future();
}

bool World::isWorkerThread() const {
	return std::this_thread::get_id() == mThread.get_id();
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
		mBox2D->QueryAABB(&q, bb);

		promise->set_value(std::move(result));
	});
	
	return promise->get_future();
}

void World::applyForceField(const Dojo::AABB& area, Group group, const Vector& force, FieldType type) {
	asyncCommand([this, area, group, force] {
		auto F = asB2Vec(force);
		auto query = AABBQuery(area, group, QUERY_FIXTURES | QUERY_PARTICLES | QUERY_PUSHABLE_ONLY).get();

		for (auto&& fixture : query.fixtures) {
			//TODO the force should be proportional to the area or to the volume
			fixture->GetBody()->ApplyForceToCenter(F, true);
		}

		for (auto&& pair : query.particles) {
			for (auto&& i : pair.second) {
				pair.first->ParticleApplyForce(i, F);
			}
		}
	});
}


Vector World::getGravity() const {
	return asVec(mBox2D->GetGravity());
}

const float MIN_SOUND_FORCE = 1.f;

float World::_closestRecentlyPlayedSound(const Vector& point) {
	float minDist = FLT_MAX;
	for (auto&& pos : mRecentlyPlayedSoundPositions) {
		minDist = std::min(minDist, pos.distanceSquared(point));
	}
	return sqrt(minDist);
}

void World::playCollisionSound(const DeferredCollision& collision, const BodyPart& part) {
	//TODO choose which sound to play... both? random? existing?
	//TODO this code doesn't belong here too much, perhaps World shouldn't know about sounds
	
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

				if (mRecentlyPlayedSoundPositions.size() > 20) {
					mRecentlyPlayedSoundPositions.pop_front();
				}
				mRecentlyPlayedSoundPositions.emplace_back(pos);
			}
		}
	}
}

Joint& World::addJoint(Unique<Joint> joint) {
	auto& ref = *joint;
	asyncCommand([this, &ref] {
		ref._init(self);
	});

	mJoints.emplace(std::move(joint));
	return ref;
}

void World::update(float dt) {
	DEBUG_ASSERT(!isWorkerThread(), "Wrong Thread");

	//remove a recently played sound
	if (mRecentlyPlayedSoundPositions.size() > 0) {
		mRemoveNextSound += dt;
		if (mRemoveNextSound > 0.5f) {
			mRecentlyPlayedSoundPositions.pop_front();
			mRemoveNextSound = 0;
		}
	}

	//play back all collisions
	DeferredCollision c;

	while (mDeferredCollisions->try_dequeue(c)) {
		auto partA = c.A.lock();
		auto partB = c.B.lock();

		if(!partA || !partB) { //one of the parts was destroyed, remove
			continue;
		}

		auto bA = partA->body;
		auto bB = partB->body;
		Vector p = asVec(c.point);

		if (bA.collisionListener) {
			bA.collisionListener->onCollision(bA, bB, c.force, p);
		}

		if (bB.collisionListener) {
			bB.collisionListener->onCollision(bB, bA, c.force, p);
		}

		playCollisionSound(c, *partA);
	}

	DeferredSensorCollision sc;

	while (mDeferredSensorCollisions->try_dequeue(sc)) {
		auto part = sc.sensor.lock();

		if (part && part->body.collisionListener) {
			part->body.collisionListener->onSensorCollision(*sc.other, part->getFixture());    //sensor collisions are not bidirectional
		}
	}

	//TODO timeslice this in some way?
	//right now it can stall the main thread with too many callbacks
	Command callback;

	while (mCallbacks->try_dequeue(callback)) {
		callback();
	}
}

Unique<Joint> World::removeJoint(Joint& joint) {
	DEBUG_ASSERT(isWorkerThread(), "Wrong Thread");

	auto elem = Dojo::SmallSet<Unique<Joint>>::find(mJoints, joint);
	DEBUG_ASSERT(elem != mJoints.end(), "Cannot remove joint as it's already removed");

	auto ptr = std::move(*elem);
	ptr->_deinit(self); //destroy the physics

	mJoints.erase(elem);
	return ptr;
}

void World::addBody(Body& body) {
	DEBUG_ASSERT(isWorkerThread(), "Wrong Thread");
	mBodies.emplace(&body);
}

void World::removeBody(Body& body) {
	DEBUG_ASSERT(isWorkerThread(), "Wrong Thread");

	//go over all joints active on this body and deactivate them
	auto joints = body.getJoints(); //copy because getjoints will remove the joints from the bodies
	for(auto&& joint : joints) {
		removeJoint(*joint);
	}

	mBodies.erase(&body);
}

void World::pause() {
	asyncCommand([this]() {
		DEBUG_ASSERT(!mSimulationPaused, "Already paused");
		mSimulationPaused = true;

		//tell all bodies they're being paused
		for (auto&& body : mBodies) {
			body->onSimulationPaused();
		}
	});
}

void World::resume() {
	asyncCommand([this]() {
		DEBUG_ASSERT(mSimulationPaused, "Already resumed");
		mSimulationPaused = false;
	});
}
