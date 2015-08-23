#pragma once

#include "common_header.h"

namespace Phys {
	class Material {
	public:
		enum State {
			Gas,
			Fluid,
			Solid
		};

		const utf::string name;
		float density = 1.f;

		//fluid parameters
		float viscosity = 1.f;
		float pressure = 0.05f;

		//solid parameters
		float friction = 0.5f;
		float restitution = 0.f;

		State state = Solid;

		Dojo::SoundSet
		* impactHard = nullptr,
		  *impactSoft = nullptr;

		Material(const utf::string& name, const Dojo::Table& desc = Dojo::Table::Empty, const Dojo::ResourceGroup* group = nullptr);
	};
}



