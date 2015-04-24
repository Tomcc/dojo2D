#pragma once

#include "common_header.h"

#include "WorldListener.h"

namespace Phys {
	class Material;
	class World;

	///TODO decouple from rendering
	class ParticleSystem : 
		public Dojo::Renderable,
		public WorldListener
	{
	public:
		const Group group;
		bool autoDeactivate = true;

		static const ParticleSystem& getFor(b2ParticleSystem* ps);

		const float damping, particleRadius;
		
		ParticleSystem(World& world, Object& parent, const Material& material, Group group, float particleSize, float damping = 0);
		virtual ~ParticleSystem();

		void addParticle(const Vector& pos, const Vector& velocity, const Dojo::Color& color, float lifetime);

		b2ParticleSystem& getParticleSystem() {
			return *particleSystem;
		}

		virtual void onAction(float dt) override;

		virtual void onPostSimulationStep() override;

	protected:
		World& world;

		const Material& material;
			
		Unique<Dojo::Mesh> mesh[2];

		Dojo::AABB activityAABB;

		b2ParticleSystem* particleSystem;
		std::atomic<bool> rebuilding = false;

		uintptr_t counter = 0;
	private:
	};
}