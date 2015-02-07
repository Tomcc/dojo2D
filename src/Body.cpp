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
	_bodyCommand([&](){
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
	world._notifyDestroyed(*this);

	world.syncCommand([&](){
		if (body) {
			world.destroyBody(*this);
			body = nullptr;
		}
		graphics = nullptr;
		staticShape = false;
		group = 0;
		particleCollisionModel = false;
	});
}

void Body::_init(Dojo::Object& obj, Dojo::Renderable* graphics, Group group, bool staticShape, bool inactive) {

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

	if (inactive)
		bodyDef.awake = bodyDef.active = false;

	bodyDef.userData = this;

	world.syncCommand([&](){
		body = world.getBox2D().CreateBody(&bodyDef);
		world.addBody(*this);
	});
}

void Body::initPhysics(Dojo::Renderable& g, Group group, bool staticShape, bool inactive) {
	_init(g, &g, group, staticShape, inactive);
}

void Body::initPhysics(Dojo::Object& obj, Group group, bool staticShape, bool inactive) {
	_init(obj, nullptr, group, staticShape, inactive);
}

// 
// void Body::initPhysics(Renderable* graphics, Body* partOf /* = nullptr */, bool fixed /* = false */) {
// 
// 	if (!fixed) {
// 		initPhysics(graphics, partOf->getGroup() );
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

void Body::_bodyCommandAsync(const World::Command& c) {
	DEBUG_ASSERT(body, "Call initPhysics first");

	if (body->IsActive() && !world.simulationPaused)
		world.asyncCommand(c);
	else
		c(); //just run this on this thread as it's out of the simulation
}

void Body::_bodyCommand(const World::Command& c) {
	DEBUG_ASSERT(body, "Call initPhysics first");

	if (body->IsActive() && !world.simulationPaused)
		world.syncCommand(c);
	else
		c(); //just run this on this thread as it's out of the simulation
}

void Body::applyForce(const Vector& force)
{
	_bodyCommandAsync([=](){
		body->ApplyForceToCenter(asB2Vec(force), true);
	});
}

void Body::applyForceAtWorldPoint(const Vector& force, const Vector& worldPoint) {
	_bodyCommandAsync([=](){
		body->ApplyForce(asB2Vec(force), asB2Vec(worldPoint), true);
	});
}

void Body::applyForceAtLocalPoint(const Vector& force, const Vector& localPoint) {
	_bodyCommandAsync([=](){
		b2Vec2 worldPoint = body->GetWorldPoint(asB2Vec(localPoint));
		body->ApplyForce(asB2Vec(force), worldPoint, true);
	});
}

void Body::applyTorque(float t) {
	_bodyCommandAsync([=](){
		body->ApplyTorque(t, true);
	});
}

void Body::setFixedRotation(bool enable) {
	_bodyCommandAsync([=](){
		body->SetFixedRotation(enable);
	});
}

void Body::forcePosition(const Vector& position) {
	_bodyCommandAsync([=](){
		auto t = body->GetTransform();
		body->SetTransform(asB2Vec(position), t.q.GetAngle());
	});
}

void Body::forceRotation(float angle) {
	_bodyCommandAsync([=](){
		auto t = body->GetTransform();
		body->SetTransform(t.p, angle);
	});
}

void Body::setTransform(const Vector& position, float angle) {
	_bodyCommandAsync([=](){
		body->SetTransform(asB2Vec(position), angle);
	});
}

void Body::forceVelocity(const Vector& velocity) {
	_bodyCommandAsync([=](){
		body->SetLinearVelocity(asB2Vec(velocity));
	});
}

void Body::setDamping(float linear, float angular) {
	_bodyCommandAsync([=](){
		body->SetLinearDamping(linear);
		body->SetAngularDamping(angular);
	});
}

float Body::getLinearDamping() const {
	DEBUG_ASSERT(body, "Call initPhysics first");

	return body->GetLinearDamping();
}

float Body::getAngularDamping() const {
	DEBUG_ASSERT(body, "Call initPhysics first");

	return body->GetAngularDamping();
}

const Vector& Body::getPosition() const
{
	DEBUG_ASSERT(graphics, "meh, shouldn't be needed");

	return graphics->position;
}

void Body::setActive() {
	DEBUG_ASSERT(body, "Call initPhysics first");

	world.asyncCommand([=](){
		if (!isStatic())
			body->SetAwake(true);
		body->SetActive(true);
	});
}


float Body::getMass() const {

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
