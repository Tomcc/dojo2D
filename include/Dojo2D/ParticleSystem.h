#pragma once

#include "common_header.h"

namespace Phys {
	class Material;
	class World;

	///TODO decouple from rendering
	class ParticleSystem : public Dojo::Renderable
	{
	public:
		const float damping;

		ParticleSystem(World& world, Object& parent, const Material& material, float damping = 0);
			
		void addParticle(const Vector& pos, const Vector& velocity, const Dojo::Color& color, float lifetime);

		b2ParticleSystem& getParticleSystem() {
			return *particleSystem;
		}

		virtual void onAction(float dt) override;

	protected:
			
		Unique<Dojo::Mesh> _mesh;

		b2ParticleSystem* particleSystem;
	private:
	};
}