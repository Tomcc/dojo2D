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

		float getMinimumDistanceTo(const Vector& pos) const;
	protected:
		b2Fixture* fixture = nullptr;
	};
}
