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
		if (cm == ContactMode::Ghost)
			deferredGhostCollisions.emplace_back(*phA, *phB, 0.f, asB2Vec(Vector::ZERO));

		return false;
	}

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

void World::AABBQuery(const Vector& min, const Vector& max, FixtureList& result) {
	class Query : public b2QueryCallback
	{
	public:
		b2AABB screen;
		FixtureList& fixtures;

		Query(const Vector& min, const Vector& max, FixtureList& fixtures) :
			fixtures(fixtures) {
			screen.lowerBound = { min.x, min.y };
			screen.upperBound = { max.x, max.y };
		}

		void run(b2World& box2D) {
			box2D.QueryAABB(this, screen);
		}

		virtual bool ReportFixture(b2Fixture* fixture) {
			fixtures.push_back(fixture);
			return true;
		}
	};

	Query q(min, max, result);
	q.run(*box2D);

}

void World::AABBQuery(const Dojo::Object& bounds, FixtureList& result) {
	AABBQuery(bounds.getWorldMin(), bounds.getWorldMax(), result);
}

Vector World::getGravity() const {
	return asVec(box2D->GetGravity());
}

void World::update(float dt) {
	deferredCollisions.clear();
	deferredGhostCollisions.clear();

	box2D->Step(timeStep, velocityIterations, positionIterations, particleIterations);

	//play back all collisions
	for (auto& c : deferredCollisions) {
		Vector p(c.point.x, c.point.y);
		c.A.onCollision(c.B, c.force, p);
		c.B.onCollision(c.A, c.force, p);
	}

	for (auto& c : deferredGhostCollisions) {
		c.A.onGhostCollision(c.B);
		c.B.onGhostCollision(c.A);
	}
}