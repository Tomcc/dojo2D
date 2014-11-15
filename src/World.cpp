#include "stdafx.h"

#include "World.h"
#include "Body.h"
#include "PhysUtil.h"

using namespace Phys;

World::World(const Vector& gravity, float timeStep, int velocityIterations, int positionIterations, int particleIterations) :
timeStep(timeStep),
velocityIterations(velocityIterations),
positionIterations(positionIterations),
particleIterations(particleIterations) {

	DEBUG_ASSERT(timeStep > 0, "Invalid timestep");

	for (int i = 0; i < GROUP_COUNT; ++i) {
		for (int j = 0; j < GROUP_COUNT; ++j)
			collideMode[j][i] = ContactMode::Normal;
	}

	//create the box 2D world
	box2D = make_unique<b2World>(asB2Vec(gravity));
	box2D->SetContactListener(this);
	box2D->SetContactFilter(this);
}

void World::setContactMode(Group A, Group B, ContactMode mode) {
	collideMode[A][B] = collideMode[B][A] = mode;
}

World::ContactMode World::getContactModeFor(Group A, Group B) const {
	auto modeA = collideMode[A][B];
	auto modeB = collideMode[B][A];

	return std::min(modeA, modeB);
}

bool World::ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB) {

	auto phA = (Body*)fixtureA->GetUserData();
	auto phB = (Body*)fixtureB->GetUserData();

	DEBUG_ASSERT(phA && phB, "Do not create Box2D objects manually");

	auto cm = getContactModeFor(phA->getGroup(), phB->getGroup());

	if (cm != ContactMode::Normal) {
		//a "ghost collision" acts like a two-way sensor
		if (cm == ContactMode::Ghost && !fixtureA->IsSensor() && !fixtureB->IsSensor()) {
			deferredSensorCollisions.emplace_back(*phB, *fixtureA);
			deferredSensorCollisions.emplace_back(*phA, *fixtureB);
		}

		return false;
	}

	//check if the sensors should collide
	if (fixtureA->IsSensor())
		deferredSensorCollisions.emplace_back(*phB, *fixtureA);
		
	if (fixtureB->IsSensor())
		deferredSensorCollisions.emplace_back(*phA, *fixtureB);

	if (fixtureA->IsSensor() && fixtureB->IsSensor())
		return false;

	bool odcA = phA->isParticle();
	bool odcB = phB->isParticle();

	if (odcA != odcB) { //only one!

		auto shouldBeBelow = odcA ? fixtureB : fixtureA;
		auto shouldBeAbove = odcA ? fixtureA : fixtureB;

		//check if they are in the correct position
		if (shouldBeBelow->GetBody()->GetPosition().y > shouldBeAbove->GetBody()->GetPosition().y - shouldBeAbove->GetAABB(0).GetExtents().y * 0.3f)
			return false;
	}

	return b2ContactFilter::ShouldCollide(fixtureA, fixtureB);
}

void World::PostSolve(b2Contact* contact, const b2ContactImpulse* impulse)
{
	b2WorldManifold worldManifold;

	DEBUG_ASSERT(contact->GetFixtureA()->GetUserData() && contact->GetFixtureB()->GetUserData(), "there is a (badly) manually-created box2D object in the world!");

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

	b2Vec2 A = bodyA.GetLinearVelocityFromWorldPoint(point);
	b2Vec2 B = bodyB.GetLinearVelocityFromWorldPoint(point);
	b2Vec2 relativeSpeed = A - B;

	DEBUG_ASSERT(phA.isStatic() || phA.getMass() > 0, "HM");
	DEBUG_ASSERT(phB.isStatic() || phB.getMass() > 0, "HM");

	float force = (relativeSpeed * phA.getMass() + relativeSpeed * phB.getMass()).Length();

	if (force > 0.1f)
		deferredCollisions.emplace_back(phA, phB, force, point);
}

RayResult World::raycast(const Vector& start, const Vector& end, Group rayBelongsToGroup /* = 0 */)
{
	RayResult result(*this);
	result.group = rayBelongsToGroup;

	box2D->RayCast(
		&result,
		{ start.x, start.y },
		{ end.x, end.y });

	if (!result.hit)
		result.position = end;

	return result;
}

void World::AABBQuery(const Vector& min, const Vector& max, Group group, BodyList& result) {

	DEBUG_ASSERT(min.x < max.x && min.y < max.y, "Invalid bounding box");
	
	auto report = [&](b2Fixture* fixture){
		auto& body = getBodyForFixture(fixture);
		auto contactMode = getContactModeFor(group, body.getGroup());
		if (contactMode == ContactMode::Normal) {
			result.push_back(&body);
		}
	};
	
	class Query : public b2QueryCallback
	{
	public:
		decltype(report)& func;
		Query(const decltype(func)& f) : func(f) {}

		virtual bool ReportFixture(b2Fixture* fixture) {
			func(fixture);
			return true;
		}
	};

	b2AABB bb;
	bb.lowerBound = asB2Vec(min);
	bb.upperBound = asB2Vec(max);
	
	Query q = report;
	box2D->QueryAABB(&q, bb);
}

void World::AABBQuery(const Dojo::Object& bounds, Group group, BodyList& result) {
	AABBQuery(bounds.getWorldMin(), bounds.getWorldMax(), group, result);
}

Vector World::getGravity() const {
	return asVec(box2D->GetGravity());
}

void World::update(float dt) {
	
	box2D->Step(timeStep, velocityIterations, positionIterations, particleIterations);

	//play back all collisions
	for (size_t i = 0; i < deferredCollisions.size(); ++i) {
		auto& c = deferredCollisions[i];
		Vector p = asVec(c.point);
		c.A->onCollision(*c.B, c.force, p);
		c.B->onCollision(*c.A, c.force, p);
	}
	deferredCollisions.clear();
	
	for (size_t i = 0; i < deferredSensorCollisions.size(); ++i) {
		auto& c = deferredSensorCollisions[i];
		getBodyForFixture(c.sensor).onSensorCollision(*c.other, *c.sensor); //sensor collisions are not bidirectional
	}

	deferredSensorCollisions.clear();
}

void Phys::World::_notifyDestroyed(Body& body) {
	//remove from the collision queues if any
	{
		for (size_t i = 0; i < deferredCollisions.size(); ++i) {
			auto& c = deferredCollisions[i];
			if (&body == c.A || &body == c.B) {
				deferredCollisions.erase(deferredCollisions.begin() + i);
				--i;
			}
		}
	}

	{
		for (size_t i = 0; i < deferredSensorCollisions.size(); ++i) {
			auto& c = deferredSensorCollisions[i];
			if (&body == c.other) {
				deferredSensorCollisions.erase(deferredSensorCollisions.begin() + i);
				--i;
			}
		}
	}

}
