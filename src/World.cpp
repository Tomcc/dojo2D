#include "stdafx.h"

#include "World.h"
#include "Body.h"
#include "PhysUtil.h"
#include "ParticleSystem.h"

using namespace Phys;

const static b2Transform B2_IDENTITY = { b2Vec2(0.f, 0.f), b2Rot(0.f) };

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
		for (int j = 0; j < GROUP_COUNT; ++j)
			collideMode[j][i] = ContactMode::Normal;
	}

	//create the box 2D world
	box2D = make_unique<b2World>(asB2Vec(gravity));
	box2D->SetContactListener(this);
	box2D->SetContactFilter(this);

	commands = make_unique<Dojo::Pipe<Job>>();
	callbacks = make_unique<Dojo::Pipe<Command>>();
	deferredCollisions = make_unique<Dojo::Pipe<DeferredCollision>>();
	deferredSensorCollisions = make_unique<Dojo::Pipe<DeferredSensorCollision>>();

	thread = std::thread([=](){
		Dojo::Timer timer;
		Job job;
		while (running) {

			//process all available commands
			while (timer.getElapsedTime() < timeStep && commands->try_dequeue(job)) {
				job.command();

				if (job.callback)
					callbacks->enqueue(std::move(job.callback));
			}


			if (timer.getElapsedTime() > timeStep)
			{
				timer.reset();
				box2D->Step(timeStep, velocityIterations, positionIterations, particleIterations);

				for (int i = 0; i < box2D->GetBodyCount(); ++i) {
					auto& body = box2D->GetBodyList()[i];

					if (body.IsAwake() && body.IsActive())
						((Body*)body.GetUserData())->updateGraphics();
				}

				for (auto&& listener : listeners)
					listener->onPostSimulationStep();
			}

			std::this_thread::yield();
		}
	});
}

World::~World() {
	running = false;
	thread.join();
}

void World::addListener(WorldListener& listener) {
	asyncCommand([&](){
		listeners.emplace(&listener);
	});
}

void World::removeListener(WorldListener& listener) {
	syncCommand([&](){
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

void World::asyncCommand(const Command& command, const Command& callback /*= Command()*/) const {
	DEBUG_ASSERT(command, "Command can't be a NOP");

	if (isWorkerThread()) {
		command();
		if (callback)
			callbacks->enqueue(callback);
	} 
	else
		commands->enqueue(command, callback);
}

void World::asyncCallback(const Command& callback) const {
	DEBUG_ASSERT(callback, "Command can't be a NOP");
	if (isWorkerThread())
		callbacks->enqueue(callback);
	else
		callback();
}

void World::sync() const
{
	if (!isWorkerThread()) {
		std::atomic<bool> done = false;
		asyncCommand([&](){
			done = true;
		});

		while (!done);
	}
}

void World::syncCommand(const Command& command) const {
	asyncCommand(command);
	sync();
}

b2Body* World::createBody(const b2BodyDef& def) {
	b2Body* res;
	syncCommand([&](){
		res = box2D->CreateBody(&def);
	});
	return res;
}

b2ParticleSystem* World::createParticleSystem(const b2ParticleSystemDef& def) {
	b2ParticleSystem* res;
	syncCommand([&](){
		res = box2D->CreateParticleSystem(&def);
	});
	return res;
}

bool World::ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB) {

	auto& phA = getBodyForFixture(fixtureA);
	auto& phB = getBodyForFixture(fixtureB);

	auto cm = getContactModeFor(phA.getGroup(), phB.getGroup());

	if (cm != ContactMode::Normal) {
		//a "ghost collision" acts like a two-way sensor
		if (cm == ContactMode::Ghost && !fixtureA->IsSensor() && !fixtureB->IsSensor()) {
			deferredSensorCollisions->enqueue( phB, phA, *fixtureA );
			deferredSensorCollisions->enqueue( phA, phB, *fixtureB );
		}

		return false;
	}

	//check if the sensors should collide
	if (fixtureA->IsSensor()) {
		deferredSensorCollisions->enqueue( phB, phA, *fixtureA );
	}
		
	if (fixtureB->IsSensor()) {
		deferredSensorCollisions->enqueue( phA, phB, *fixtureB );
	}

	if (fixtureA->IsSensor() && fixtureB->IsSensor())
		return false;

	bool odcA = phA.isParticle();
	bool odcB = phB.isParticle();

	if (odcA != odcB) { //only one!

		auto shouldBeBelow = odcA ? fixtureB : fixtureA;
		auto shouldBeAbove = odcA ? fixtureA : fixtureB;

		//check if they are in the correct position
		if (shouldBeBelow->GetBody()->GetPosition().y > shouldBeAbove->GetBody()->GetPosition().y - shouldBeAbove->GetAABB(0).GetExtents().y * 0.3f)
			return false;
	}

	return b2ContactFilter::ShouldCollide(fixtureA, fixtureB);
}

bool World::ShouldCollide(b2Fixture* fixture, b2ParticleSystem* particleSystem, int32 particleIndex)
{
	auto& body = getBodyForFixture(fixture);
	auto& ps = ParticleSystem::getFor(particleSystem);

	if (getContactModeFor(body.getGroup(), ps.group) != ContactMode::Normal)
		return false;
	else {
		return b2ContactFilter::ShouldCollide(fixture, particleSystem, particleIndex);
	}
}

void World::PostSolve(b2Contact* contact, const b2ContactImpulse* impulse)
{
	b2WorldManifold worldManifold;

	auto& phA = getBodyForFixture(contact->GetFixtureA());
	auto& phB = getBodyForFixture(contact->GetFixtureB());

	contact->GetWorldManifold(&worldManifold);

	const b2Body& bodyA = *contact->GetFixtureA()->GetBody();
	const b2Body& bodyB = *contact->GetFixtureB()->GetBody();

	b2Vec2 point;
	if (contact->GetManifold()->pointCount == 1)
		point = worldManifold.points[0];
	else
		point = (worldManifold.points[0] + worldManifold.points[1]) * 0.5f;
	
	DEBUG_ASSERT(phA.isStatic() || phA.getMass() > 0, "HM");
	DEBUG_ASSERT(phB.isStatic() || phB.getMass() > 0, "HM");

	auto& N = worldManifold.normal;
//	b2Vec2 T = { N.y, -N.x };
	b2Vec2 F = { 0, 0 };
	for (int i = 0; i < impulse->count; ++i)
		F += impulse->normalImpulses[i] * N;

	float force = F.Length();
	if (force > 0.1f) {
		deferredCollisions->enqueue( phA, phB, force, point );
	}
}

void World::asyncRaycast(const Vector& start, const Vector& end, Group rayBelongsToGroup, RayResult& result) const
{
	result.group = rayBelongsToGroup;
	asyncCommand([=, &result](){
		box2D->RayCast(
			&result,
			{ start.x, start.y },
			{ end.x, end.y });

		if (!result.hit)
			result.position = end;

		result.dist = start.distance(result.position);
	});
}

bool World::isWorkerThread() const
{
	return std::this_thread::get_id() == thread.get_id();
}

RayResult World::raycast(const Vector& start, const Vector& end, Group rayBelongsToGroup /*= 0*/) const
{
	RayResult result(*this);
	asyncRaycast(start, end, rayBelongsToGroup, result);
	sync();
	return result;
}

bool World::_AABBQuery(const Vector& min, const Vector& max, Group group, BodyList* resultBody, FixtureList* resultFixture, bool precise /*= false*/) const
{
	DEBUG_ASSERT(min.x < max.x && min.y < max.y, "Invalid bounding box");

	bool empty = true;
	syncCommand([&](){
		b2PolygonShape aabbShape;
		if (precise) {
			Vector dim = max - min;
			Vector center = (max + min) * 0.5f;
			aabbShape.SetAsBox(dim.x, dim.y, asB2Vec(center), 0);
		}

		auto report = [&](b2Fixture* fixture){
			if (!fixture->IsSensor()) {
				auto& body = getBodyForFixture(fixture);
				auto contactMode = getContactModeFor(group, body.getGroup());
				if (contactMode == ContactMode::Normal) {

					if (!precise || shapesOverlap(aabbShape, *fixture)) {
						empty = false;

						if (resultBody)
							resultBody->insert(&body);

						else if (resultFixture)
							resultFixture->push_back(fixture);
						else
							return false; //stop search immediately
					}
				}
			}

			return true;
		};

		class Query : public b2QueryCallback
		{
		public:
			decltype(report)& func;
			Query(const decltype(func)& f) : func(f) {}

			virtual bool ReportFixture(b2Fixture* fixture) {
				return func(fixture);
			}
		};

		b2AABB bb;
		bb.lowerBound = asB2Vec(min);
		bb.upperBound = asB2Vec(max);

		Query q = report;
		box2D->QueryAABB(&q, bb);
	});

	return empty;
}

void World::AABBQuery(const Vector& min, const Vector& max, Group group, BodyList& result, bool precise) const {
	_AABBQuery(min, max, group, &result, nullptr, precise);
}

void World::AABBQuery(const Vector& min, const Vector& max, Group group, FixtureList& result, bool precise /*= false*/) const {
	_AABBQuery(min, max, group, nullptr, &result, precise);
}


bool World::AABBQueryEmpty(const Vector& min, const Vector& max, Group group, bool precise /*= false*/) const {
	return _AABBQuery(min, max, group, nullptr, nullptr, precise);
}


Vector World::getGravity() const {
	return asVec(box2D->GetGravity());
}

void World::update(float dt) {

	DeferredCollision c;
	DeferredSensorCollision sc;
	Command callback;

	//play back all collisions
	while (deferredCollisions->try_dequeue(c)) {
		Vector p = asVec(c.point);
		c.A->onCollision(*c.B, c.force, p);
		c.B->onCollision(*c.A, c.force, p);
	}

	while (deferredSensorCollisions->try_dequeue(sc))
		getBodyForFixture(sc.sensor).onSensorCollision(*sc.other, *sc.sensor); //sensor collisions are not bidirectional

	while (callbacks->try_dequeue(callback))
		callback();
}

void World::_notifyDestroyed(Body& body) {
	//remove from the collision queues if any

	//TODO
// 	{
// 		for (size_t i = 0; i < deferredCollisions.size(); ++i) {
// 			auto& c = deferredCollisions[i];
// 			if (&body == c.A || &body == c.B) {
// 				deferredCollisions.erase(deferredCollisions.begin() + i);
// 				--i;
// 			}
// 		}
// 	}
// 
// 	{
// 		for (size_t i = 0; i < deferredSensorCollisions.size(); ++i) {
// 			auto& c = deferredSensorCollisions[i];
// 			if (&body == c.other || &body == c.me) {
// 				deferredSensorCollisions.erase(deferredSensorCollisions.begin() + i);
// 				--i;
// 			}
// 		}
// 	}

}

