#include "stdafx.h"

#include "ParticleSystem.h"

#include "Material.h"
#include "PhysUtil.h"
#include "World.h"

using namespace Phys;
using namespace Dojo;

Phys::ParticleSystem::ParticleSystem(World& world, Object& parent, const Material& material, float particleRadius, float damping /*= 0*/) : 
Dojo::Renderable(&parent, Vector::ZERO),
damping(damping),
particleRadius(particleRadius),
_mesh(new Mesh()) {
	DEBUG_ASSERT(particleRadius > 0, "Invalid particle size");

	setMesh(_mesh.get());

	mesh->setDynamic(true);
	mesh->setTriangleMode(TriangleMode::TriangleList);
	mesh->setVertexFields({ VertexField::Position2D, VertexField::Color, VertexField::UV0 });

	//HACK
	setTexture(parent.getGameState()->getTexture("particle"));

	//LIQUIDTHING
	b2ParticleSystemDef particleSystemDef;
	particleSystemDef.radius = 0.05f;
	particleSystemDef.destroyByAge = false;
	particleSystem = world.getBox2D().CreateParticleSystem(&particleSystemDef);
}

void Phys::ParticleSystem::addParticle(const Vector& pos, const Vector& velocity, const Color& color, float lifetime) {
	DEBUG_ASSERT(lifetime > 0, "Invalid lifetime");

	b2ParticleDef particle;
	particle.flags = b2_springParticle | b2_colorMixingParticle;
	particle.position = asB2Vec(pos);
	particle.velocity = asB2Vec(velocity);
	particle.color = b2Color(color.r, color.g, color.b);
	particle.lifetime = lifetime;
	particle.userData = (void*)rand();

	particleSystem->CreateParticle(particle);
}

void ParticleSystem::onAction(float dt) {
	Object::onAction(dt);

	setVisible(particleSystem->GetParticleCount() > 0);

	if(isVisible()) {
		mesh->begin(particleSystem->GetParticleCount());

		auto position = particleSystem->GetPositionBuffer();
		auto color = particleSystem->GetColorBuffer();
		auto userData = (uintptr_t*)particleSystem->GetUserDataBuffer();
		auto velocity = particleSystem->GetVelocityBuffer();

		for (int i = 0; i < particleSystem->GetParticleCount(); ++i, ++position, ++color, ++velocity, ++userData) {
			b2Color c1 = color->GetColor();
			//Color c(c1.r, c1.g, c1.b, 1.f);
			auto& c = Color::RED;

			auto baseIdx = mesh->getVertexCount();

			int hash = ((*userData * 0x1f1f1f1f) >> 1) & 0xf;
			
			float r = particleRadius;
			if (hash < 5 && hash > 0)
				r -= 0.01f * hash;

			mesh->vertex(position->x - r, position->y - r);
			mesh->color(c);
			mesh->uv(0, 0);
			mesh->vertex(position->x + r, position->y - r);
			mesh->color(c);
			mesh->uv(1, 0);
			mesh->vertex(position->x - r, position->y + r);
			mesh->color(c);
			mesh->uv(0, 1);
			mesh->vertex(position->x + r, position->y + r);
			mesh->color(c);
			mesh->uv(1, 1);

			mesh->quad(baseIdx, baseIdx + 2, baseIdx + 1, baseIdx + 3);

			*velocity *= (1.f - damping);
		}

		mesh->end();
	}
}
