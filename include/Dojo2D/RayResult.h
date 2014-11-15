#pragma once

#include "common_header.h"

namespace Phys {
	class Body;
	class World;

	struct RayResult : public b2RayCastCallback {
		bool hit = false;
		Vector position;
		Vector normal;
		Body *hitBody = nullptr;
		Group group = 0;

		RayResult(World& world) :
			world(world) {

		}
			
		float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction) override;

		float32 ReportParticle(const b2ParticleSystem* particleSystem, int32 index, const b2Vec2& point, const b2Vec2& normal, float32 fraction) override {
			return -1; //ignore all particles
		}

	protected:
		World& world;
	};
}



