#include "stdafx.h"

#include "BodyPart.h"

Phys::BodyPart::BodyPart(const Material& material) :
material(material) {

}

b2Fixture& Phys::BodyPart::getFixture() const {
	while (!fixture) //the fixture must be in flight on the other thread, wait for it
		std::this_thread::yield();

	return *fixture;
}
