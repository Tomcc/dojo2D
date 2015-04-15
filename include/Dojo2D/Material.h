#pragma once

#include "common_header.h"

namespace Phys {
	class Material {
	public:
		const Dojo::String name;
		float density = 1.f;

		//fluid parameters
		float viscosity = 1.f;
		float pressure = 0.05f;

		//solid parameters
		float friction = 0.5f;
		float restitution = 0.f;

		Material(const Dojo::String& name) :
			name(name) {

			DEBUG_ASSERT(name.size() > 0, "Invalid material name");
		}
	};
}



