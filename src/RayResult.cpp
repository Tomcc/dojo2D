#include "RayResult.h"
#include "PhysUtil.h"
#include "World.h"
#include "Body.h"
#include "BodyPart.h"

using namespace Phys;

float32 RayResult::ReportFixture(b2Fixture* fixture, const b2Vec2& P, const b2Vec2& N, float32 fraction)
{
	if (fixture->IsSensor())
		return -1; //ignore

	if (world->getContactModeFor(getBodyForFixture(fixture).getGroup(), group) != ContactMode::Normal)
		return -1; //ignore as these two groups can't see each other

	hitFixture = fixture;

	position = asVec(P);
	normal = asVec(N);
	hit = true;

	return fraction; //stop
}

Body* RayResult::getHitBody() const {
	return hitFixture ? &getBodyForFixture(hitFixture) : nullptr;
}

const Material* RayResult::getHitMaterial() const {
	return hitFixture ? &getPartForFixture(hitFixture).material : nullptr;
}

bool RayResult::ShouldQueryParticleSystem(const b2ParticleSystem* particleSystem) {
	return false;
}

