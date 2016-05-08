#include "BodyPart.h"
#include "PhysUtil.h"

using namespace Phys;

BodyPart::BodyPart(Body& body, const Material& material, Group group) 
	: material(material)
	, body(body)
	, group(group) {

}

b2Fixture& BodyPart::getFixture() const {
	while (fixture.is_none()) { //the fixture must be in flight on the other thread, wait for it
		std::this_thread::yield();
	}

	return fixture.unwrap();
}

float BodyPart::getMinimumDistanceTo(const Vector& pos) const {
	auto& fixture = getFixture();
	b2Vec2 normal;
	float res;
	fixture.GetShape()->ComputeDistance(
		fixture.GetBody()->GetTransform(),
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

optional_ref<b2PolygonShape> Phys::BodyPart::getPolyShape() const {
	if (getFixture().GetType() != b2Shape::e_polygon) {
		return{};
	}
	return static_cast<b2PolygonShape&>(getShape());
}

std::vector<Vector> Phys::BodyPart::getWorldContour() const {
	auto& shape = getPolyShape().unwrap();
	auto body = getFixture().GetBody();

	//TODO do without poly shape?

	std::vector<Vector> contour;
	contour.reserve(shape.m_count);
	for (auto i : range(shape.m_count)) {
		contour.push_back(Phys::asVec(body->GetWorldPoint(shape.m_vertices[i])));
	}
	return contour;
}

float Phys::BodyPart::getMass() const {
	b2MassData data;
	getFixture().GetMassData(&data);
	return data.mass;
}

b2FixtureDef Phys::BodyPart::makeDefinition() const {
	auto& fixture = getFixture();

	b2FixtureDef def;
	def.shape = fixture.GetShape();
	def.userData = fixture.GetUserData();
	def.friction = fixture.GetFriction();
	def.restitution = fixture.GetRestitution();
	def.density = fixture.GetDensity();
	def.isSensor = fixture.IsSensor();

	return def;
}

void Phys::BodyPart::_resetFixture(b2Fixture& fix) {
	fixture = fix;
}
