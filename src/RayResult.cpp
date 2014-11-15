#include "stdafx.h"

#include "RayResult.h"
#include "PhysUtil.h"
#include "World.h"
#include "Body.h"

using namespace Phys;

float32 RayResult::ReportFixture(b2Fixture* fixture, const b2Vec2& P, const b2Vec2& N, float32 fraction)
{
	if (fixture->IsSensor())
		return -1; //ignore

	hitBody = (Body*)fixture->GetUserData();

	if (world.getContactModeFor(hitBody->getGroup(), group) != World::ContactMode::Normal)
		return -1; //ignore as these two groups can't see each other

	position = asVec(P);
	normal = asVec(N);
	hit = true;

	return fraction; //stop
}

