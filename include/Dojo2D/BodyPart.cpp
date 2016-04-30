#include "BodyPart.h"
#include "PhysUtil.h"

using namespace Phys;

BodyPart::BodyPart(Body& body, const Material& material, Group group) 
	: material(material)
	, body(body)
	, group(group) {

}

b2Fixture& BodyPart::getFixture() const {
	while (not fixture) { //the fixture must be in flight on the other thread, wait for it
		std::this_thread::yield();
	}

	return *fixture;
}

float BodyPart::getMinimumDistanceTo(const Vector& pos) const {
	b2Vec2 normal;
	float res;
	fixture->GetShape()->ComputeDistance(
		fixture->GetBody()->GetTransform(),
		asB2Vec(pos),
		&res,
		&normal,
		0);

	return res;
}

b2Shape& Phys::BodyPart::getShape() const {
	DEBUG_ASSERT(getFixture().GetShape(), "No shape available");
	return *getFixture().GetShape();
}
