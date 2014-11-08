#include "stdafx.h"

#include "ParticleSystem.h"

#include "Material.h"
#include "PhysUtil.h"
#include "World.h"

using namespace Phys;
using namespace Dojo;

ParticleSystem::ParticleSystem(World& world, Dojo::Object& parent, const Material& material, float damping) : 
Dojo::Renderable(&parent, Vector::ZERO),
damping(damping),
_mesh(new Mesh()) {
	mesh = _mesh.get();

	mesh->setDynamic(true);
	mesh->setTriangleMode(TriangleMode::TriangleList);
	mesh->setVertexFields({ VertexField::Position2D, VertexField::Color });

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

	particleSystem->CreateParticle(particle);
}

void ParticleSystem::onAction(float dt) {
	Object::onAction(dt);

	setVisible(particleSystem->GetParticleCount() > 0);

	if(isVisible()) {

		const float r = 0.07f;

		mesh->begin(particleSystem->GetParticleCount());

		auto position = particleSystem->GetPositionBuffer();
		auto color = particleSystem->GetColorBuffer();

		auto velocity = particleSystem->GetVelocityBuffer();

		for (int i = 0; i < particleSystem->GetParticleCount(); ++i, ++position, ++color, ++velocity) {
			b2Color c1 = color->GetColor();
			Color c(c1.r, c1.g, c1.b, 1.f);

			auto baseIdx = mesh->getVertexCount();

			mesh->vertex(position->x - r, position->y - r);
			mesh->color(c);
			mesh->vertex(position->x + r, position->y - r);
			mesh->color(c);
			mesh->vertex(position->x - r, position->y + r);
			mesh->color(c);
			mesh->vertex(position->x + r, position->y + r);
			mesh->color(c);

			mesh->quad(baseIdx, baseIdx + 2, baseIdx + 1, baseIdx + 3);

			*velocity *= (1.f - damping);
		}

		mesh->end();
	}
}
