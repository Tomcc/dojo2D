#include "stdafx.h"

#include "ParticleSystemRenderer.h"
#include "ParticleSystem.h"

using namespace Dojo;
using namespace Phys;

ParticleSystemRenderer::ParticleSystemRenderer(ParticleSystem& ps) :
Renderable(ps),
particleSystem(ps) {

}

void ParticleSystemRenderer::setAABB(const AABB& box) {
	worldBB = box;
}

void ParticleSystemRenderer::update(float dt) {
	//TODO move all the tessellation sheananigans here
}
