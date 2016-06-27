#pragma once

namespace Phys {
	class BodyPart;

	enum class FieldType {
		ApplyToSurface,
		ApplyToVolume,
		Constant
	};

	class ForceField {
	public:
		FieldType type = FieldType::Constant;
		bool relative = false;
		Vector mForce;

		void applyTo(const BodyPart& part, optional_ref<const BodyPart> relativeTo) const;

	private:
	};
}

