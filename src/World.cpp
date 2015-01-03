#include "stdafx.h"

#include "World.h"
#include "Body.h"
#include "PhysUtil.h"

using namespace Phys;

const static b2Transform B2_IDENTITY = { b2Vec2(0.f, 0.f), b2Rot(0.f) };

bool Phys::World::shapesOverlap(const b2Shape& s1, const b2Transform& t1, const b2Shape& s2, const b2Transform& t2) {
	return b2TestOverlap(&s1, 0, &s2, 0, t1, t2);
}

bool World::shapesOverlap(const b2Shape& shape, const b2Fixture& fixture) {
	return shapesOverlap(shape, B2_IDENTITY, *fixture.GetShape(), fixture.GetBody()->GetTransform());
}


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
			deferredSensorCollisions.emplace_back(*phB, *phA, *fixtureA);
			deferredSensorCollisions.emplace_back(*phA, *phB, *fixtureB);
		}

		return false;
	}

	//check if the sensors should collide
	if (fixtureA->IsSensor())
		deferredSensorCollisions.emplace_back(*phB, *phA, *fixtureA);
		
	if (fixtureB->IsSensor())
		deferredSensorCollisions.emplace_back(*phA, *phB, *fixtureB);

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
	
	DEBUG_ASSERT(phA.isStatic() || phA.getMass() > 0, "HM");
	DEBUG_ASSERT(phB.isStatic() || phB.getMass() > 0, "HM");

	auto& N = worldManifold.normal;
//	b2Vec2 T = { N.y, -N.x };
	b2Vec2 F = { 0, 0 };
	for (int i = 0; i < impulse->count; ++i)
		F += impulse->normalImpulses[i] * N;

	float force = F.Length();
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

void World::AABBQuery(const Vector& min, const Vector& max, Group group, BodyList& result, bool precise, bool emptyCheckOnly) {

	DEBUG_ASSERT(min.x < max.x && min.y < max.y, "Invalid bounding box");

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
					result.insert(&body);

					if (emptyCheckOnly)
						return false; //stop search
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
}

void World::AABBQuery(const Dojo::Object& bounds, Group group, BodyList& result, bool precise) {
	AABBQuery(bounds.getWorldMin(), bounds.getWorldMax(), group, result, precise);
}

bool Phys::World::AABBQueryEmpty(const Vector& min, const Vector& max, Group group, bool precise /*= false*/, const Body* except /*= nullptr*/) {
	static BodyList list;
	list.clear();
	AABBQuery(min, max, group, list, precise, true);

	auto count = list.size() - (except ? list.count(except) : 0);
	return count == 0;
}


Vector World::getGravity() const {
	return asVec(box2D->GetGravity());
}

void World::update(float dt) {
	
	box2D->Step(dt, velocityIterations, positionIterations, particleIterations);

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
			if (&body == c.other || &body == c.me) {
				deferredSensorCollisions.erase(deferredSensorCollisions.begin() + i);
				--i;
			}
		}
	}

}
