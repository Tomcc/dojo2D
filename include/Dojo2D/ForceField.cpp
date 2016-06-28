#include "ForceField.h"

#include "BodyPart.h"
#include "Body.h"
#include "World.h"

Phys::Vector Phys::ForceField::getBaseForceFor(const BodyPart& part) const {
	switch (mType)
	{
	case Phys::FieldType::Constant:
		return mConstantForce;
	case FieldType::WeightProportional: {
		Vector F = -part.body.getMass() * part.body.getWorld().getGravity() * mMultiplier;
		auto len = F.length();
		if(len > mMaxForce) {
			F *= mMaxForce / len;
		}
		return F;
	}
	default:
		FAIL("Invalid type");
	}

}

void Phys::ForceField::applyTo(const BodyPart& part, optional_ref<const BodyPart> relativeTo) const {
	auto F = getBaseForceFor(part);
	if(F == Vector::Zero) {
		return;
	}

	if(mRelative) {
		auto len = F.length();
		F = relativeTo.unwrap().body.getLocalDirection(F / len) * len;
		F.x *= -1;
	}

	//TODO apply to the center of the bodypart or something??
	part.body.applyForce(F);

	part.body.getObject().get<Dojo::Renderable>().startFade(Dojo::Color::Red, Dojo::Color::Gray, 0.1f);

	//TODO particles!!!
}
