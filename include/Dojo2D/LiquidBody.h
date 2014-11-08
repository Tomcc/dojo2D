#pragma once

#include "common_header.h"

namespace Phys {
	class Material;
	class World;

	class LiquidBody : public Dojo::Renderable
	{
	public:
		LiquidBody(World& world, Object& parent, const Material& material);
			
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