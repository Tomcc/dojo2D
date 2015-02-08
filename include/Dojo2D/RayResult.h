#pragma once

#include "common_header.h"

namespace Phys {
	class Body;
	class World;
	class Material;

	struct RayResult : public b2RayCastCallback {
		bool hit = false;
		Vector position;
		Vector normal;
		b2Fixture *hitFixture = nullptr;
		Group group = 0;
		float dist;

		RayResult(const World& world) :
			world(&world) {

		}
			
		float32 ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float32 fraction) override;

		float32 ReportParticle(const b2ParticleSystem* particleSystem, int32 index, const b2Vec2& point, const b2Vec2& normal, float32 fraction) override {
			return -1; //ignore all particles
		}

		Body* getHitBody() const;

		const Material* getHitMaterial() const;

		virtual bool ShouldQueryParticleSystem(const b2ParticleSystem* particleSystem) override;

	protected:
		const World* world;
	};
}



