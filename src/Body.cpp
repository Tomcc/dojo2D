#include "Body.h"
#include "Material.h"
#include "PhysUtil.h"
#include "World.h"
#include "BodyPart.h"

using namespace Phys;

Body::Body(Dojo::Object& object, World& world) :
	world(world),
	object(object) {

}

Body::~Body() {
	destroyPhysics();
	world.sync(); //cannot let the body be destroyed before the physics :(
	//TODO find a way to give the body back to world?
}

BodyPart& Phys::Body::_addShape(Shared<b2Shape> shape, const Material& material, bool sensor) {
	//TODO use a unique pointer when capture-by-move is available

	parts.emplace_back(make_unique<BodyPart>(material));

	auto& part = *parts.back();
	world.asyncCommand([this, &part, &material, sensor, shape]() {

		b2FixtureDef fixtureDef;

		fixtureDef.shape = shape.get();
		fixtureDef.density = material.density;
		fixtureDef.friction = material.friction;
		fixtureDef.restitution = material.restitution;
		fixtureDef.filter = b2Filter();
		fixtureDef.filter.groupIndex = group;
		fixtureDef.isSensor = sensor;

		fixtureDef.userData = (void*)&part;

		part.fixture = body->CreateFixture(&fixtureDef);
	});

	return part;
}

BodyPart& Body::addPolyShape(const Material& material, const b2Vec2* points, int count, bool sensor /*= false*/) {

	DEBUG_ASSERT(count > 0, "Wrong vertex count");
	DEBUG_ASSERT(count < b2_maxPolygonVertices, "This box shape has too many vertices!");

	auto shape = make_shared<b2PolygonShape>();
	shape->Set(points, count);

	return _addShape(shape, material, sensor);
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

	auto circle = make_shared<b2CircleShape>();
	circle->m_radius = radius;
	circle->m_p.x = center.x;
	circle->m_p.y = center.y;

	return _addShape(circle, material, sensor);
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

	//appear like the body is destroyed instantly on the main thread
	auto oldBody = body;
	body = nullptr;
	world.asyncCommand([&, oldBody]() {
		if (oldBody) {
			world.destroyBody(*this);
			world.getBox2D().DestroyBody(oldBody);
		}
	});
}

void Body::initPhysics(Group group, bool staticShape, bool inactive) {
	this->group = group;

	b2BodyDef bodyDef;
	bodyDef.type = staticShape ? b2_staticBody : b2_dynamicBody;

	bodyDef.position = asB2Vec(object.position);
	bodyDef.angle = object.getRoll();

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

	world.asyncCommand([this, bodyDef]() {
		body = world.getBox2D().CreateBody(&bodyDef);
		world.addBody(*this);
	});
}

void Body::onSimulationPaused() {
	if (auto graphics = object.getRenderable()) {
		graphics->speed = Vector::ZERO;
	}
}

void Body::updateGraphics() {
	//TODO this should just set the interpolation target rather than the actual transform?
	if (auto graphics = object.getRenderable()) {
		DEBUG_ASSERT(body, "Call initPhysics first");

		auto& t = body->GetTransform();

		graphics->position = asVec(t.p);
		graphics->speed = asVec(body->GetLinearVelocity());

		if (body->IsFixedRotation()) {
			body->SetTransform(t.p, graphics->getRoll());
		}
		else
			graphics->setRoll(Radians(t.q.GetAngle()));
	}
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

void Phys::Body::forceRotation(Radians angle) {
	world.asyncCommand([=]() {
		DEBUG_ASSERT(body, "Call initPhysics first");
		auto t = body->GetTransform();
		body->SetTransform(t.p, angle);
	});
}

void Phys::Body::setTransform(const Vector& position, Radians angle) {
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
	DEBUG_ASSERT(body, "Call initPhysics first");

	return body->GetLinearDamping();
}

float Body::getAngularDamping() const {
	DEBUG_ASSERT(body, "Call initPhysics first");

	return body->GetAngularDamping();
}

Dojo::Vector Phys::Body::getPosition() const {
	DEBUG_ASSERT(body, "meh, shouldn't be needed");

	return asVec(body->GetPosition());
}

void Body::setActive() {
	DEBUG_ASSERT(body, "Call initPhysics first");

	world.asyncCommand([=]() {
		DEBUG_ASSERT(body, "Call initPhysics first");
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
