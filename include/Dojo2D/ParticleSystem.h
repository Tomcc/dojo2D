#pragma once

#include "common_header.h"

namespace Phys {
	class Material;
	class World;

	///TODO decouple from rendering
	class ParticleSystem : public Dojo::Renderable
	{
	public:
		const Group group;
		bool autoDeactivate = true;

		static const ParticleSystem& getFor(b2ParticleSystem* ps);

		const float damping, particleRadius;
		
		ParticleSystem(World& world, Object& parent, const Material& material, Group group, float particleSize, float damping = 0);
			
		void addParticle(const Vector& pos, const Vector& velocity, const Dojo::Color& color, float lifetime);

		b2ParticleSystem& getParticleSystem() {
			return *particleSystem;
		}

		virtual void onAction(float dt) override;

	protected:
		World& world;

		float warmupTime = 0.f;
		const Material& material;
			
		Unique<Dojo::Mesh> _mesh;

		AABB activityAABB;

		b2ParticleSystem* particleSystem;

		uintptr_t counter = 0;
	private:
	};
}