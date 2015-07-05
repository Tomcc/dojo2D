#include "Body.h"
#include "Material.h"
#include "PhysUtil.h"
#include "World.h"
#include "BodyPart.h"

using namespace Phys;

Body::Body(Dojo::Object& object, World& world, Group group, bool staticShape, bool inactive) :
	Component(object),
	world(world) {

	this->group = group;

	b2BodyDef bodyDef;
	bodyDef.type = staticShape ? b2_staticBody : b2_dynamicBody;

	bodyDef.position = asB2Vec(self.position);
	bodyDef.angle = self.getRoll();

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

	world.asyncCommand([this, bodyDef, &world]() {
		body = world.getBox2D().CreateBody(&bodyDef);
		world.addBody(*this);
	});
}

Body::~Body() {
	DEBUG_ASSERT(!body, "This body was not properly disposed of before destruction");
}

BodyPart& Phys::Body::_addShape(Shared<b2Shape> shape, const Material& material, bool sensor) {
	auto elem = parts.emplace(make_unique<BodyPart>(*this, material));

	//TODO make the pointer Unique when at some point MSVC won't try to copy the lambda
	auto& part = **elem;
	auto f = [this, &part, &material, sensor, lshape = std::move(shape)]() {

		b2FixtureDef fixtureDef;

		fixtureDef.shape = lshape.get();
		fixtureDef.density = material.density;
		fixtureDef.friction = material.friction;
		fixtureDef.restitution = material.restitution;
		fixtureDef.filter = b2Filter();
		fixtureDef.filter.groupIndex = group;
		fixtureDef.isSensor = sensor;

		fixtureDef.userData = (void*)&part;

		part.fixture = body->CreateFixture(&fixtureDef);
	};
	world.asyncCommand(std::move(f));

	return part;
}

void Phys::Body::removeShape(BodyPart& part) {
	auto elem = Dojo::SmallSet<Unique<BodyPart>>::find(parts, part);
	DEBUG_ASSERT(elem != parts.end(), "Part already removed");

	//remove the part from the parts known to this thread, give it to a lambda
	//TODO make the pointer Unique when at some point MSVC won't try to copy the lambda
	//auto temp = std::move(*elem);

	Shared<BodyPart> temp = std::move(*elem);
	parts.erase(elem);
	world.asyncCommand([this, part = std::move(temp)]{
		body->DestroyFixture(&part->getFixture());
	});
}

BodyPart& Body::addPolyShape(const Material& material, const b2Vec2* points, int count, bool sensor /*= false*/) {

	DEBUG_ASSERT(count > 0, "Wrong vertex count");
	DEBUG_ASSERT(count < b2_maxPolygonVertices, "This box shape has too many vertices!");

	auto shape = make_unique<b2PolygonShape>();
	shape->Set(points, count);

	return _addShape(std::move(shape), material, sensor);
}

BodyPart& Body::addPolyShape(const Material& material, const std::vector<b2Vec2>& points /*= nullptr*/, bool sensor /*= false */) {
	return addPolyShape(material, points.data(), points.size(), sensor);
}

BodyPart& Body::addBoxShape(const Material& material, const Vector& dimensions, const Vector& center /*= Vector::ZERO*/, bool sensor /*= false*/) {

	DEBUG_ASSERT(dimensions.x >= 0 && dimensions.y >= 0, "Invalid dimensions");

	Vector min = center - dimensions * 0.5f;
	Vector max = center + dimensions * 0.5f;

	const b2Vec2 points[4] = {
		{min.x, min.y},
		{min.x, max.y},
		{max.x, max.y},
		{max.x, min.y}
	};

	return addPolyShape(material, points, 4, sensor);
}

BodyPart& Body::addCircleShape(const Material& material, float radius, const Vector& center, bool sensor /*= false*/) {
	DEBUG_ASSERT(radius > 0, "Invalid radius");

	auto circle = make_unique<b2CircleShape>();
	circle->m_radius = radius;
	circle->m_p.x = center.x;
	circle->m_p.y = center.y;

	return _addShape(std::move(circle), material, sensor);
}

BodyPart& Body::addCapsuleShape(const Material& material, const Vector& dimensions, const Vector& center, bool sensor /*= false*/) {

	Vector halfSize = dimensions * 0.5f;
	Vector offset(0, halfSize.y - halfSize.x);

	//create physics
	addCircleShape(material, halfSize.x, center + offset, sensor);
	return addCircleShape(material, halfSize.x, center - offset, sensor);
}

void Body::destroyPhysics() {
	world._notifyDestroyed(*this);

	staticShape = false;
	group = 0;
	particleCollisionModel = false;

	world.asyncCommand([&]() {
		if (body) {
			world.removeBody(*this);
			world.getBox2D().DestroyBody(body);
			body = nullptr;
		}
	});
}

void Body::onSimulationPaused() {
	self.speed = Vector::Zero;
}

void Body::updateObject() {
	//TODO this should just set the interpolation target rather than the actual transform?
	auto& t = body->GetTransform();

	self.position = asVec(t.p);
	self.speed = asVec(body->GetLinearVelocity());

	if (body->IsFixedRotation()) {
		body->SetTransform(t.p, self.getRoll());
	}
	else
		self.setRoll(Radians(t.q.GetAngle()));
}

void Body::applyForce(const Vector& force) {
	DEBUG_ASSERT(force.isValid(), "This will hang up b2d mang");
	world.asyncCommand([=]() {
		DEBUG_ASSERT(body, "Call initPhysics first");
		body->ApplyForceToCenter(asB2Vec(force), true);
	});
}

void Body::applyForceAtWorldPoint(const Vector& force, const Vector& worldPoint) {
	DEBUG_ASSERT(force.isValid(), "This will hang up b2d mang");
	world.asyncCommand([=]() {
		DEBUG_ASSERT(body, "Call initPhysics first");
		body->ApplyForce(asB2Vec(force), asB2Vec(worldPoint), true);
	});
}

void Body::applyForceAtLocalPoint(const Vector& force, const Vector& localPoint) {
	DEBUG_ASSERT(force.isValid(), "This will hang up b2d mang");
	world.asyncCommand([=]() {
		DEBUG_ASSERT(body, "Call initPhysics first");
		b2Vec2 worldPoint = body->GetWorldPoint(asB2Vec(localPoint));
		body->ApplyForce(asB2Vec(force), worldPoint, true);
	});
}

void Body::applyTorque(float t) {
	world.asyncCommand([=]() {
		DEBUG_ASSERT(body, "Call initPhysics first");
		body->ApplyTorque(t, true);
	});
}

void Body::setFixedRotation(bool enable) {
	world.asyncCommand([=]() {
		DEBUG_ASSERT(body, "Call initPhysics first");
		body->SetFixedRotation(enable);
	});
}

void Body::forcePosition(const Vector& position) {
	world.asyncCommand([=]() {
		DEBUG_ASSERT(body, "Call initPhysics first");
		auto t = body->GetTransform();
		body->SetTransform(asB2Vec(position), t.q.GetAngle());
	});
}

void Body::forceRotation(Radians angle) {
	world.asyncCommand([=]() {
		DEBUG_ASSERT(body, "Call initPhysics first");
		auto t = body->GetTransform();
		body->SetTransform(t.p, angle);
	});
}

void Body::setTransform(const Vector& position, Radians angle) {
	world.asyncCommand([=]() {
		DEBUG_ASSERT(body, "Call initPhysics first");
		body->SetTransform(asB2Vec(position), angle);
	});
}

void Body::forceVelocity(const Vector& velocity) {
	world.asyncCommand([=]() {
		DEBUG_ASSERT(body, "Call initPhysics first");
		body->SetLinearVelocity(asB2Vec(velocity));
	});
}

void Body::setDamping(float linear, float angular) {
	world.asyncCommand([=]() {
		DEBUG_ASSERT(body, "Call initPhysics first");
		body->SetLinearDamping(linear);
		body->SetAngularDamping(angular);
	});
}

float Body::getLinearDamping() const {
	_waitForBody();

	return body->GetLinearDamping();
}

float Body::getAngularDamping() const {
	_waitForBody();

	return body->GetAngularDamping();
}

Dojo::Vector Body::getPosition() const {
	_waitForBody();
	return asVec(body->GetPosition());
}

void Body::setActive() {
	world.asyncCommand([=]() {
		DEBUG_ASSERT(body, "Call initPhysics first");
		if (!isStatic())
			body->SetAwake(true);
		body->SetActive(true);
	});
}

float Body::getMass() const {
	_waitForBody();
	return body->GetMass();
}

Vector Body::getLocalPoint(const Vector& worldPosition) const {
	_waitForBody();
	return asVec(body->GetLocalPoint(asB2Vec(worldPosition)));
}

Vector Body::getWorldPoint(const Vector& localPosition) const {
	_waitForBody();
	return asVec(body->GetWorldPoint(asB2Vec(localPosition)));
}

Vector Body::getVelocity() const {
	_waitForBody();
	return asVec(body->GetLinearVelocity());
}

Vector Body::getVelocityAtLocalPoint(const Vector& localPoint) const {
	_waitForBody();
	return asVec(body->GetLinearVelocityFromLocalPoint(asB2Vec(localPoint)));
}

float Body::getMinimumDistanceTo(const Vector& point) const {
	_waitForBody();
	
	float min = FLT_MAX;
	for (auto&& part : parts) {
		min = std::min(min, part->getMinimumDistanceTo(point));
	}
	return min;
}

void Phys::Body::_waitForBody() const {
	//if the body is not yet here, assume it could be somewhere in the command pipeline
	if (!body)
		world.sync();

	DEBUG_ASSERT(body, "Call initPhysics first!");
}
