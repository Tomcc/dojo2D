#include "stdafx.h"

#include "ParticleSystem.h"

#include "Material.h"
#include "PhysUtil.h"
#include "World.h"

using namespace Phys;
using namespace Dojo;

const ParticleSystem& ParticleSystem::getFor(b2ParticleSystem* ps)
{
	DEBUG_ASSERT(ps, "no particle system");
	DEBUG_ASSERT(ps->GetParticleCount() > 0, "This PS cannot collide");

	return *(ParticleSystem*)ps->GetUserDataBuffer()[0];
}

ParticleSystem::ParticleSystem(World& world, Object& parent, const Material& material, Group group, float particleRadius, float damping /*= 0*/) : 
Dojo::Renderable(&parent, Vector::ZERO),
material(material),
damping(damping),
particleRadius(particleRadius),
world(world),
group(group),
_mesh(new Mesh()) {
	DEBUG_ASSERT(particleRadius > 0, "Invalid particle size");

	setMesh(_mesh.get());

	mesh->setDynamic(true);
	mesh->setTriangleMode(TriangleMode::TriangleList);
	mesh->setVertexFields({ VertexField::Position2D, VertexField::Color, VertexField::UV0 });
	//mesh->setIndexByteSize(4);

	//LIQUIDTHING
	b2ParticleSystemDef particleSystemDef;
	particleSystemDef.radius = particleRadius; 
	particleSystemDef.destroyByAge = true;

	particleSystemDef.density = material.density;
  	particleSystemDef.pressureStrength = material.pressure;
	particleSystemDef.dampingStrength = material.friction;

	particleSystem = world.createParticleSystem(particleSystemDef);

	particleSystem->SetDamping(damping);

	world.addListener(*this);
}

ParticleSystem::~ParticleSystem() {
	world.removeListener(*this);
}

void ParticleSystem::addParticle(const Dojo::Vector& pos, const Dojo::Vector& velocity, const Color& color, float lifetime) {
	DEBUG_ASSERT(lifetime > 0, "Invalid lifetime");

	b2ParticleDef particle;
	particle.flags = b2_waterParticle | b2_colorMixingParticle;
	particle.position = asB2Vec(pos);
	particle.velocity = asB2Vec(velocity);
	particle.color = b2Color(color.r, color.g, color.b);
	particle.lifetime = lifetime;
	particle.userData = this;

	world.asyncCommand([=](){
		particleSystem->CreateParticle(particle);
	});
}

void ParticleSystem::onAction(float dt) {
	Object::onAction(dt);
	
	advanceFade(dt);
}

void ParticleSystem::onPostSimulationStep()
{
	b2AABB b2bb;
	particleSystem->ComputeAABB(&b2bb);

	worldBB.max = asVec(b2bb.upperBound);
	worldBB.min = asVec(b2bb.lowerBound);

	activityAABB = worldBB.grow(5);

	auto& viewport = *getGameState()->getViewport();

	//suspend the particlesystem when it's too far from the player
	bool active = (!autoDeactivate) || (viewport.isInViewRect(activityAABB));
	particleSystem->SetPaused(!active);

	//only show when active, visible and has particles
	setVisible(active && viewport.isInViewRect(*this) && particleSystem->GetParticleCount() > 0);

	if (isVisible() && !building) {
		building = true;
		mesh->begin(particleSystem->GetParticleCount());

		auto position = particleSystem->GetPositionBuffer();
		auto color = particleSystem->GetColorBuffer();
		//auto userData = (uintptr_t*)particleSystem->GetUserDataBuffer();
		auto velocity = particleSystem->GetVelocityBuffer();

		for (int i = 0; i < particleSystem->GetParticleCount(); ++i, ++position, ++color, ++velocity) {
			//int hash = ((*userData * 0x1f1f1f1f) >> 1) & 0xf;

			if (viewport.isInViewRect(Vector{ position->x, position->y })) {
				b2Color c1 = color->GetColor();
				Color c(c1.r, c1.g, c1.b, 1.f);

				auto baseIdx = mesh->getVertexCount();

				float r = particleRadius*1.5f;
				// 			if (hash < 5 && hash > 0)
				// 				r -= 0.03f * hash;

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
			}
		}

		//fire a callback to rebuild the mesh
		world.asyncCallback([this](){
			mesh->end();
			building = false;
			setVisible(isVisible() && mesh->getVertexCount() > 0);
		});
	}

}
