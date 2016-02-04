#include "ParticleSystem.h"

#include "Material.h"
#include "PhysUtil.h"
#include "World.h"
#include "ParticleSystemRenderer.h"

using namespace Phys;
using namespace Dojo;

const ParticleSystem& ParticleSystem::getFor(b2ParticleSystem* ps) {
	DEBUG_ASSERT(ps, "no particle system");
	DEBUG_ASSERT(ps->GetParticleCount() > 0, "This PS cannot collide");

	return *(ParticleSystem*)ps->GetUserDataBuffer()[0];
}

ParticleSystem::ParticleSystem(World& world, Object& parent, Dojo::RenderLayer::ID layer, const Material& material, Group group, float particleSize, float damping /*= 0*/) :
	Object(parent, Vector::Zero),
	damping(damping),
	world(world),
	group(group),
	material(material) {
	DEBUG_ASSERT(particleSize > 0, "Invalid particle size");

	addComponent([&] {
		auto r = make_unique<ParticleSystemRenderer>(*this, layer);
		r->setTexture(getGameState().getTexture("particle").unwrap());
		r->setVisible(false);
		r->setShader(getGameState().getShader("textured_mult_color").unwrap());
		return r;
	}());

	world.asyncCommand([this, damping, particleSize, &material, &world]() {
		b2ParticleSystemDef particleSystemDef;
		particleSystemDef.radius = particleSize;
		particleSystemDef.destroyByAge = true;

		particleSystemDef.density = material.density;
		particleSystemDef.pressureStrength = material.pressure;
		particleSystemDef.dampingStrength = material.friction;

		particleSystem = world.getBox2D().CreateParticleSystem(&particleSystemDef);
		particleSystem->SetDamping(damping);
		world.addListener(*this);
	});
}

ParticleSystem::~ParticleSystem() {
	world.removeListener(*this);
	world.sync();
}

ParticleSystem::Particle::Particle(const Dojo::Vector& pos, const Dojo::Vector& velocity, const Dojo::Color& color, float lifetime) {
	DEBUG_ASSERT(lifetime > 0, "Invalid lifetime");

	def.flags = b2_waterParticle | b2_colorMixingParticle;
	def.position = asB2Vec(pos);
	def.velocity = asB2Vec(velocity);
	def.color = b2Color(color.r, color.g, color.b);
	def.lifetime = lifetime;
	def.userData = this;
}

void ParticleSystem::addParticles(const ParticleList& particles) {
	world.asyncCommand([ = ]() {
		for (auto&& particle : particles) {
			particleSystem->CreateParticle(particle.def);
		}
	});
}

void ParticleSystem::addParticles(ParticleList&& rhs) {
	world.asyncCommand([this, particles = std::move(rhs)]() {
		for (auto&& particle : particles) {
			particleSystem->CreateParticle(particle.def);
		}
	});
}

void ParticleSystem::applyForceField(const Dojo::Vector& force) {
	world.asyncCommand([this, force] {
		for (auto i : range(0, particleSystem->GetParticleCount())) {
			particleSystem->ParticleApplyForce(i, asB2Vec(force));
		}
	});
}

void ParticleSystem::onPostSimulationStep() {
	//TODO split this between this and its Renderable instead of calling setAABB
	b2AABB b2bb;
	particleSystem->ComputeAABB(&b2bb);

	mSimulationAABB = AABB{
		asVec(b2bb.lowerBound),
		asVec(b2bb.upperBound)
	};

	auto& viewport = getGameState().getViewport().unwrap();

	//suspend the particlesystem when it's too far from the player
	mSimulating = (!autoDeactivate) || (viewport.isInViewRect(mSimulationAABB.grow(3)));
	particleSystem->SetPaused(!mSimulating);

}

