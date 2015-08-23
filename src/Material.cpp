#include "Material.h"

using namespace Phys;
using namespace Dojo;

Material::Material(const utf::string& name, const Table& desc, const ResourceGroup* group) :
	name(name) {
	DEBUG_ASSERT(name.not_empty(), "Invalid name");

	density = desc.getNumber("density", density);
	friction = desc.getNumber("friction", friction);
	restitution = desc.getNumber("restitution", restitution);

	//state-specific stuff
	if (desc.getBool("fluid")) {
		state = Material::Fluid;

		viscosity = desc.getNumber("viscosity", viscosity);
		pressure = desc.getNumber("pressure", pressure);
	}
	else if (desc.getBool("gas")) {
		state = Material::Gas;
	}
	else {
		state = Material::Solid;
	}

	//load sounds if available
	if (group && desc.existsAs("soundPrefix", Table::FieldType::String)) {
		auto prefix = desc.getString("soundPrefix");

		impactHard = group->getSound(prefix + "_impact_hard");
		impactSoft = group->getSound(prefix + "_impact_soft");
	}
}
