#pragma once

#include "common_header.h"

namespace Phys {
	class ParticleSystem;

	class ParticleSystemRenderer : public Dojo::Renderable
	{
	public:
		static const int ID = 4;

		ParticleSystemRenderer(ParticleSystem& ps, Dojo::RenderLayer::ID layer);

		void setAABB(const Dojo::AABB& box);

		virtual void update(float dt) override;

	protected:
		ParticleSystem& particleSystem;
	private:
	};

}

