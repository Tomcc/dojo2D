#include "stdafx.h"

#include "Body.h"
#include "Material.h"
#include "PhysUtil.h"
#include "World.h"

using namespace Phys;

Body::Body(World& world) :
world(world) {

}


Body::~Body() {
	destroyPhysics();
}

b2Fixture& Body::_addShape(b2Shape& shape, const Material& material, bool sensor) {
	b2FixtureDef fixtureDef;

	fixtureDef.shape = &shape;
	fixtureDef.density = material.density;
	fixtureDef.friction = material.friction;
	fixtureDef.restitution = material.restitution;
	fixtureDef.filter = b2Filter();
	fixtureDef.filter.groupIndex = group;
	fixtureDef.isSensor = sensor;

	fixtureDef.userData = this;

	return *body->CreateFixture(&fixtureDef);
}

b2Fixture& Body::addPolyShape(const Material& material, const b2Vec2* points, int count, bool sensor /*= false*/) {

	DEBUG_ASSERT(count > 0, "Wrong vertex count");
	DEBUG_ASSERT(count < b2_maxPolygonVertices, "This box shape has too many vertices!");

	b2PolygonShape shape;
	shape.Set(points, count);

	return _addShape(shape, material, sensor);
}

b2Fixture& Body::addPolyShape(const Material& material, const std::vector<b2Vec2>& points /*= nullptr*/, bool sensor /*= false */) {
	return addPolyShape(material, points.data(), points.size(), sensor);
}

b2Fixture& Body::addBoxShape(const Material& material, const Vector& dimensions, const Vector& center /*= Vector::ZERO*/, bool sensor /*= false*/) {

	DEBUG_ASSERT(dimensions.x >= 0 && dimensions.y >= 0, "Invalid dimensions");

	Vector min = center - dimensions * 0.5f;
	Vector max = center + dimensions * 0.5f;

	const b2Vec2 points[4] = {
		b2Vec2(min.x, min.y),
		b2Vec2(min.x, max.y),
		b2Vec2(max.x, max.y),
		b2Vec2(max.x, min.y)
	};

	return addPolyShape(material, points, 4, sensor);
}

b2Fixture& Body::addCircleShape(const Material& material, float radius, const Vector& center, bool sensor /*= false*/) {
	DEBUG_ASSERT(radius > 0, "Invalid radius");

	b2CircleShape circle;
	circle.m_radius = radius;
	circle.m_p.x = center.x;
	circle.m_p.y = center.y;

	return _addShape(circle, material, sensor);
}

b2Fixture& Body::addCapsuleShape(const Material& material, const Vector& dimensions, const Vector& center, bool sensor /*= false*/) {

	Vector halfSize = dimensions * 0.5f;
	Vector offset(0, halfSize.y - halfSize.x);

	//create physics
	addCircleShape(material, halfSize.x, center + offset, sensor);
	return addCircleShape(material, halfSize.x, center - offset, sensor);
}

void Body::destroyPhysics() {
	
	graphics = nullptr;
	if (body) {
		body->GetWorld()->DestroyBody(body);
		body = nullptr;
	}

	staticShape = false;
	group = 0;
	particleCollisionModel = false;
}

void Body::_init(Dojo::Object& obj, Dojo::Renderable* graphics, Phys::Group group, bool staticShape) {

	this->graphics = graphics;
	this->group = group;

	b2BodyDef bodyDef;
	bodyDef.type = staticShape ? b2_staticBody : b2_dynamicBody;

	bodyDef.position = { obj.position.x, obj.position.y };
	bodyDef.angle = obj.getRoll() * Dojo::Math::EULER_TO_RADIANS;

	if (staticShape) {
		bodyDef.awake = false;
		this->staticShape = staticShape;
	}
	else {
		bodyDef.angularDamping = 0.1f;
		bodyDef.linearDamping = 0.1f;
		//bodyDef.bullet = true;
	}

	bodyDef.userData = this;

	body = world.getBox2D().CreateBody(&bodyDef);
}

void Body::initPhysics(Dojo::Renderable& g, Phys::Group group, bool staticShape) {
	_init(g, &g, group, staticShape);
}

void Body::initPhysics(Dojo::Object& obj, Phys::Group group, bool staticShape) {
	_init(obj, nullptr, group, staticShape);
}

// 
// void Body::initPhysics(Renderable* graphics, Body* partOf /* = nullptr */, bool fixed /* = false */) {
// 
// 	if (!fixed) {
// 		initPhysics(graphics, partOf->getPhys::Group() );
// 	}
// 	else {
// 		DEBUG_TODO;
// 	}
// }

void Body::updateGraphics() {
	if (graphics) {
		DEBUG_ASSERT(body, "Call initPhysics first");

		auto& t = body->GetTransform();

		graphics->position.x = t.p.x;
		graphics->position.y = t.p.y;

		auto speed = body->GetLinearVelocity();
		graphics->speed = Vector(speed.x, speed.y);

		if (body->IsFixedRotation()) {
			body->SetTransform(t.p, graphics->getRoll());
		}
		else
			graphics->setRotation(Vector(0, 0, t.q.GetAngle()));
	}
}

void Body::applyForce(const Vector& force)
{
	DEBUG_ASSERT(body, "Call initPhysics first");

	body->ApplyForceToCenter(asB2Vec(force), true);
}

void Body::applyForceAtWorldPoint(const Vector& force, const Vector& worldPoint) {
	DEBUG_ASSERT(body, "Call initPhysics first");

	body->ApplyForce(asB2Vec(force), asB2Vec(worldPoint), true);
}

void Body::applyForceAtLocalPoint(const Vector& force, const Vector& localPoint) {
	DEBUG_ASSERT(body, "Call initPhysics first");

	b2Vec2 worldPoint = body->GetWorldPoint(asB2Vec(localPoint));
	body->ApplyForce(asB2Vec(force), worldPoint, true);
}

void Body::applyTorque(float t) {
	DEBUG_ASSERT(body, "Call initPhysics first");

	body->ApplyTorque(t, true);
}

void Phys::Body::setFixedRotation(bool enable) {
	DEBUG_ASSERT(body, "Call initPhysics first");

	body->SetFixedRotation(enable);
}

void Phys::Body::forceRotation(float angle) {
	DEBUG_ASSERT(body, "Call initPhysics first");

	auto t = body->GetTransform();
	body->SetTransform(t.p, angle);
}

float Body::getMass() const {
	DEBUG_ASSERT(body, "Call initPhysics first");

	return body->GetMass();
}

Vector Body::getPosition() const {
	DEBUG_ASSERT(body, "Call initPhysics first");
	auto b2p = body->GetWorldCenter();
	return Vector(b2p.x, b2p.y);
}

Vector Body::getLocalPoint(const Vector& worldPosition) const {
	DEBUG_ASSERT(body, "Call initPhysics first");

	return asVec(body->GetLocalPoint(asB2Vec(worldPosition)));
}

Vector Body::getWorldPoint(const Vector& localPosition) const {
	DEBUG_ASSERT(body, "Call initPhysics first");
	return asVec(body->GetWorldPoint(asB2Vec(localPosition)));
}

Vector Body::getVelocity() const {
	DEBUG_ASSERT(body, "Call initPhysics first");

	return asVec(body->GetLinearVelocity());
}

Vector Body::getVelocityAtLocalPoint(const Vector& localPoint) const {
	DEBUG_ASSERT(body, "Call initPhysics first");

	return asVec(body->GetLinearVelocityFromLocalPoint(asB2Vec(localPoint)));
}
