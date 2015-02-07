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

	fixtureDef.userData = (void*)&material;

	b2Fixture* fixture = nullptr;
	world.syncCommand([&](){
		fixture = body->CreateFixture(&fixtureDef);
	});
	return *fixture;
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
			{ min.x, min.y },
			{ min.x, max.y },
			{ max.x, max.y },
			{ max.x, min.y }
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
		world._notifyDestroyed(*this);

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
	bodyDef.angle = obj.getRoll();

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

	body = world.createBody(bodyDef);
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

	world.asyncCommand([=](){
		body->ApplyForceToCenter(asB2Vec(force), true);
	});
}

void Body::applyForceAtWorldPoint(const Vector& force, const Vector& worldPoint) {
	DEBUG_ASSERT(body, "Call initPhysics first");

	world.asyncCommand([=](){
		body->ApplyForce(asB2Vec(force), asB2Vec(worldPoint), true);
	});
}

void Body::applyForceAtLocalPoint(const Vector& force, const Vector& localPoint) {
	DEBUG_ASSERT(body, "Call initPhysics first");

	world.asyncCommand([=](){
		b2Vec2 worldPoint = body->GetWorldPoint(asB2Vec(localPoint));
		body->ApplyForce(asB2Vec(force), worldPoint, true);
	});
}

void Body::applyTorque(float t) {
	DEBUG_ASSERT(body, "Call initPhysics first");

	world.asyncCommand([=](){
		body->ApplyTorque(t, true);
	});
}

void Phys::Body::setFixedRotation(bool enable) {
	DEBUG_ASSERT(body, "Call initPhysics first");

	world.asyncCommand([=](){
		body->SetFixedRotation(enable);
	});
}

void Phys::Body::forcePosition(const Vector& position) {
	world.asyncCommand([=](){
		auto t = body->GetTransform();
		body->SetTransform(asB2Vec(position), t.q.GetAngle());
	});
}

void Phys::Body::forceRotation(float angle) {
	DEBUG_ASSERT(body, "Call initPhysics first");

	world.asyncCommand([=](){
		auto t = body->GetTransform();
		body->SetTransform(t.p, angle);
	});
}

void Phys::Body::setTransform(const Vector& position, float angle) {
	world.asyncCommand([=](){
		body->SetTransform(asB2Vec(position), angle);
	});
}

void Phys::Body::forceVelocity(const Vector& velocity) {
	world.asyncCommand([=](){
		body->SetLinearVelocity(asB2Vec(velocity));
	});
}

void Phys::Body::setDamping(float linear, float angular) {
	DEBUG_ASSERT(body, "Call initPhysics first");

	world.asyncCommand([=](){
		body->SetLinearDamping(linear);
		body->SetAngularDamping(angular);
	});
}

float Phys::Body::getLinearDamping() const {
	DEBUG_ASSERT(body, "Call initPhysics first");

	return body->GetLinearDamping();
}

float Phys::Body::getAngularDamping() const {
	DEBUG_ASSERT(body, "Call initPhysics first");

	return body->GetAngularDamping();
}

const Vector& Phys::Body::getPosition() const
{
	DEBUG_ASSERT(graphics, "meh, shouldn't be needed");

	return graphics->position;
}

float Body::getMass() const {
	DEBUG_ASSERT(body, "Call initPhysics first");

	return body->GetMass();
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
