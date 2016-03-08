#include "Body.h"
#include "Material.h"
#include "PhysUtil.h"
#include "World.h"
#include "BodyPart.h"

using namespace Phys;

Body::Body(Dojo::Object& object, World& world, Group group, bool staticShape, bool inactive) 
	: Component(object)
	, world(world)
	, group(group) {

	b2BodyDef bodyDef;
	bodyDef.type = staticShape ? b2_staticBody : b2_dynamicBody;

	bodyDef.position = asB2Vec(object.position);
	bodyDef.angle = object.getRoll();

	if (staticShape) {
		bodyDef.awake = false;
		self.staticShape = staticShape;
	}
	else {
		bodyDef.angularDamping = 0.1f;
		bodyDef.linearDamping = 0.1f;
		//bodyDef.bullet = true;
	}

	if (inactive) {
		bodyDef.awake = bodyDef.active = false;
	}

	bodyDef.userData = this;

	world.asyncCommand([this, bodyDef, &world]() {
		body = *world.getBox2D().CreateBody(&bodyDef);
		world.addBody(self);
	});
}

Body::~Body() {

}

BodyPart& Phys::Body::_addShape(Shared<b2Shape> shape, const Material& material, bool sensor) {
	auto elem = parts.emplace(make_unique<BodyPart>(self, material));

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

		part.fixture = body.unwrap().CreateFixture(&fixtureDef);
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
	world.asyncCommand([this, part = std::move(temp)] {
		body.unwrap().DestroyFixture(&part->getFixture());
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
	world._notifyDestroyed(self);

	staticShape = false;
	group = 0;
	particleCollisionModel = false;

	world.asyncCommand([&] {
		if (body.is_some()) {
			world.removeBody(self);
			world.getBox2D().DestroyBody(body.to_raw_ptr());
			body = {};
		}
	});
}

void Body::onDestroy(Unique<Component> myself) {
	if(body.is_some()) {
		destroyPhysics();
		//assign it to a task so that it can survive until it's destroyed
		world.asyncCommand([owned = Shared<Component>(std::move(myself))]() mutable {
			owned = {};
		});
	}
}

void Body::onSimulationPaused() {
	object.speed = Vector::Zero;
}

void Body::updateObject() {
	//TODO this should just set the interpolation target rather than the actual transform?
	auto& t = body.unwrap().GetTransform();

	object.position = asVec(t.p);
	object.speed = asVec(body.unwrap().GetLinearVelocity());

	if (body.unwrap().IsFixedRotation()) {
		body.unwrap().SetTransform(t.p, object.getRoll());
	}
	else {
		object.setRoll(Radians(t.q.GetAngle()));
	}
}

void Body::applyForce(const Vector& force) {
	DEBUG_ASSERT(force.isValid(), "This will hang up b2d mang");
	world.asyncCommand([ = ]() {
		body.unwrap().ApplyForceToCenter(asB2Vec(force), true);
	});
}

void Body::applyForceAtWorldPoint(const Vector& force, const Vector& worldPoint) {
	DEBUG_ASSERT(force.isValid(), "This will hang up b2d mang");
	world.asyncCommand([ = ]() {
		body.unwrap().ApplyForce(asB2Vec(force), asB2Vec(worldPoint), true);
	});
}

void Body::applyForceAtLocalPoint(const Vector& force, const Vector& localPoint) {
	DEBUG_ASSERT(force.isValid(), "This will hang up b2d mang");
	world.asyncCommand([ = ]() {
		b2Vec2 worldPoint = body.unwrap().GetWorldPoint(asB2Vec(localPoint));
		body.unwrap().ApplyForce(asB2Vec(force), worldPoint, true);
	});
}

void Body::applyTorque(float t) {
	world.asyncCommand([ = ]() {
		body.unwrap().ApplyTorque(t, true);
	});
}

void Body::setFixedRotation(bool enable) {
	world.asyncCommand([ = ]() {
		body.unwrap().SetFixedRotation(enable);
	});
}

void Body::forcePosition(const Vector& position) {
	world.asyncCommand([ = ]() {
		auto t = body.unwrap().GetTransform();
		body.unwrap().SetTransform(asB2Vec(position), t.q.GetAngle());
	});
}

void Body::forceRotation(Radians angle) {
	world.asyncCommand([ = ]() {
		auto t = body.unwrap().GetTransform();
		body.unwrap().SetTransform(t.p, angle);
	});
}

void Body::setTransform(const Vector& position, Radians angle) {
	world.asyncCommand([ = ]() {
		body.unwrap().SetTransform(asB2Vec(position), angle);
	});
}

void Body::forceVelocity(const Vector& velocity) {
	world.asyncCommand([ = ]() {
		body.unwrap().SetLinearVelocity(asB2Vec(velocity));
	});
}

void Body::setDamping(float linear, float angular) {
	world.asyncCommand([ = ]() {
		body.unwrap().SetLinearDamping(linear);
		body.unwrap().SetAngularDamping(angular);
	});
}

float Body::getLinearDamping() const {
	return _waitForBody().GetLinearDamping();
}

float Body::getAngularDamping() const {
	return _waitForBody().GetAngularDamping();
}

Dojo::Vector Body::getPosition() const {
	return asVec(_waitForBody().GetPosition());
}

void Body::setActive() {
	world.asyncCommand([ = ]() {
		if (!isStatic()) {
			body.unwrap().SetAwake(true);
		}

		body.unwrap().SetActive(true);
	});
}

float Body::getMass() const {
	return _waitForBody().GetMass();
}

Vector Body::getLocalPoint(const Vector& worldPosition) const {
	return asVec(_waitForBody().GetLocalPoint(asB2Vec(worldPosition)));
}

Vector Body::getWorldPoint(const Vector& localPosition) const {
	return asVec(_waitForBody().GetWorldPoint(asB2Vec(localPosition)));
}

Vector Body::getVelocity() const {
	return asVec(_waitForBody().GetLinearVelocity());
}

Vector Body::getVelocityAtLocalPoint(const Vector& localPoint) const {
	return asVec(_waitForBody().GetLinearVelocityFromLocalPoint(asB2Vec(localPoint)));
}

float Body::getMinimumDistanceTo(const Vector& point) const {
	_waitForBody();
	float min = FLT_MAX;
	for (auto&& part : parts) {
		min = std::min(min, part->getMinimumDistanceTo(point));
	}

	return min;
}

b2Body& Phys::Body::_waitForBody() const
{
	//if the body is not yet here, assume it could be somewhere in the command pipeline
	if (body.is_none()) {
		world.sync();
	}

	return body.unwrap();
}

float Body::getWeight() const {
	return world.getGravity().length() * getMass();
}

bool Body::isPushable() const {
	return pushable && _waitForBody().IsActive() && !isStatic();
}
