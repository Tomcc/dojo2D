#pragma once

#include "common_header.h"

#include "WorldListener.h"

namespace Phys {
	class Material;
	class World;

	///TODO decouple from rendering
	class ParticleSystem :
		public Dojo::Object,
		public WorldListener {
	public:
		struct Particle {
			b2ParticleDef def;

			Particle(const Vector& pos, const Vector& velocity, const Dojo::Color& color, float lifetime);
		};

		typedef std::vector<Particle> ParticleList;

		const Group group;
		bool autoDeactivate = true;

		static const ParticleSystem& getFor(b2ParticleSystem* ps);

		const float damping;

		ParticleSystem(World& world, Object& parent, Dojo::RenderLayer::ID layer, const Material& material, Group group, float particleSize, float damping = 0);
		virtual ~ParticleSystem();

		void addParticles(const ParticleList& particles);
		void addParticles(ParticleList&& particles);

		void applyForceField(const Dojo::Vector& force);
		
		b2ParticleSystem& getParticleSystem() {
			return *particleSystem;
		}

		World& getWorld() const {
			return world;
		}

		bool isSimulating() const {
			return mSimulating;
		}


		const Dojo::AABB& getSimulationAABB() const {
			return mSimulationAABB;
		}

		virtual void onPostSimulationStep() override;

	protected:
		const Material& material;

		World& world;

		Dojo::AABB mSimulationAABB;

		b2ParticleSystem* particleSystem;

		uintptr_t counter = 0;
		std::atomic<bool> mSimulating = false;
	private:
	};
}