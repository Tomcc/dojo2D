#pragma once

#include "common_header.h"

namespace Phys {
	class Material {
	public:
		float density = 1.f;

		//fluid parameters
		float viscosity = 1.f;

		//solid parameters
		float friction = 0.5f;
		float restitution = 0.f;

	};
}



