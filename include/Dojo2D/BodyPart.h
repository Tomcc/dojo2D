#pragma once

#include "common_header.h"

namespace Phys {
	class Material;

	class BodyPart {
	public:
		friend class Body;

		const Material& material;

		BodyPart(const Material& material);

		b2Fixture& getFixture() const;

	protected:
		b2Fixture* fixture = nullptr;
	};
}
