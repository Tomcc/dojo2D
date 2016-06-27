#include "ForceField.h"

#include "BodyPart.h"
#include "Body.h"

void Phys::ForceField::applyTo(const BodyPart& part, optional_ref<const BodyPart> relativeTo) const {
	if(mForce == Vector::Zero) {
		return;
	}
	
	Vector F;
	if(relative) {
		auto len = mForce.length();
		F = relativeTo.unwrap().body.getLocalDirection(mForce / len) * len;
		F.x *= -1;
	}
	else {
		F = mForce;
	}

	switch (type)
	{
	case Phys::FieldType::ApplyToSurface:
	case Phys::FieldType::ApplyToVolume:
		DEBUG_TODO; //figure out how to scale the force
	case Phys::FieldType::Constant:
		break;
	default:
		FAIL("Invalid type");
	}

	//TODO apply to the center of the bodypart or something??
	part.body.applyForce(F);

	//TODO particles!!!
}
