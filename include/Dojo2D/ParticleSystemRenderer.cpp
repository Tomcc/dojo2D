#include "stdafx.h"

#include "ParticleSystemRenderer.h"
#include "ParticleSystem.h"

using namespace Dojo;
using namespace Phys;

Phys::ParticleSystemRenderer::ParticleSystemRenderer(ParticleSystem& ps, Dojo::RenderLayer::ID layer) :
Renderable(ps, layer),
particleSystem(ps) {

}

void ParticleSystemRenderer::setAABB(const AABB& box) {
	worldBB = box;
}

void ParticleSystemRenderer::update(float dt) {
	//TODO move all the tessellation sheananigans here
}
