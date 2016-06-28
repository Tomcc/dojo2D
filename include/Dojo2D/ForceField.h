#pragma once

namespace Phys {
	class BodyPart;

	enum class FieldType {
		Constant,
		WeightProportional //the force applied is proportional to the weight up to a limit
	};

	class ForceField {
	public:
		FieldType mType = FieldType::Constant;
		bool mRelative = false;
		Vector mConstantForce;
		float mMaxForce = FLT_MAX;
		float mMultiplier = 3.f;

		void applyTo(const BodyPart& part, optional_ref<const BodyPart> relativeTo) const;

		Vector getBaseForceFor(const BodyPart& part) const;
	private:
	};
}

