#include "stdafx.h"

#include "RayResult.h"
#include "PhysUtil.h"
#include "World.h"
#include "Body.h"

using namespace Phys;

float32 RayResult::ReportFixture(b2Fixture* fixture, const b2Vec2& P, const b2Vec2& N, float32 fraction)
{
	hitActor = (Phys::Body*)fixture->GetUserData();

	if (world.getContactModeFor(hitActor->getGroup(), group) != World::ContactMode::Normal)
		return -1; //ignore as these two groups can't see each other

	position = Phys::asVec(P);
	normal = Phys::asVec(N);
	hit = true;

	return fraction; //stop
}

