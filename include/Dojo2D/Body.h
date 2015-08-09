#pragma once

#include "common_header.h"

namespace Phys {

	class Material;
	class World;
	class BodyPart;
	class Body;

	class CollisionListener {
	public:
		virtual void onCollision(Phys::Body& me, Phys::Body& other, float force, const Vector& point) {}
		virtual void onSensorCollision(Body& other, b2Fixture& sensor) {};
	};

	class Body : public Dojo::Component	{
	public:
		static const int ID = 1;

		bool pushable = true;
		CollisionListener* collisionListener = nullptr;

		Body(Dojo::Object& object, World& world, Group group, bool staticShape = false, bool inactive = false);

		virtual ~Body();

		BodyPart& addPolyShape(const Material& material, const b2Vec2* points, int count, bool sensor = false);
		BodyPart& addPolyShape(const Material& material, const std::vector<b2Vec2>& points, bool sensor = false);
		BodyPart& addBoxShape(const Material& material, const Vector& dimensions, const Vector& center = Vector::Zero, bool sensor = false);
		BodyPart& addCircleShape(const Material& material, float radius, const Vector& center = Vector::Zero, bool sensor = false);
		BodyPart& addCapsuleShape(const Material& material, const Vector& dimensions, const Vector& center = Vector::Zero, bool sensor = false);

		void removeShape(BodyPart& part);

		///Removes physical behaviors from this object
		void destroyPhysics();

		void enableParticleCollisions() {
			particleCollisionModel = true;
		}

		void setFixedRotation(bool enable);

		void applyForce(const Vector& force);
		void applyForceAtWorldPoint(const Vector& force, const Vector& localPoint);
		void applyForceAtLocalPoint(const Vector& force, const Vector& localPoint);

		void applyTorque(float t);

		void forcePosition(const Vector& position);
		void forceVelocity(const Vector& velocity);
		void forceRotation(Radians angle);

		void setActive();

		virtual float getMass() const;

		Vector getLocalPoint(const Vector& worldPosition) const;
		Vector getWorldPoint(const Vector& localPosition) const;

		Vector getVelocity() const;
		Vector getVelocityAtLocalPoint(const Vector& localPoint) const;

		Vector getPosition() const;

		float getLinearDamping() const;
		float getAngularDamping() const;

		void setDamping(float linear, float angular);

		void setTransform(const Vector& position, Radians angle);

		Group getGroup() const {
			return group;
		}

		b2Body* getB2Body() const {
			return body;
		}

		bool isStatic() const {
			return staticShape;
		}

		bool isParticle() const {
			return particleCollisionModel;
		}

		void onSimulationPaused();

		void updateObject();

		World& getWorld() const {
			return world;
		}

		float getMinimumDistanceTo(const Vector& pos) const;

		virtual void onDispose() override {
			destroyPhysics();
		}

		virtual bool canDestroy() const override {
			//wait until the World has really destroyed the body
			return body == nullptr;
		}

	protected:
		World& world;

		b2Body* body = nullptr;
		Group group = Group::invalid();
		bool staticShape = false;

		Dojo::SmallSet<Unique<BodyPart>> parts;

		BodyPart& _addShape(Shared<b2Shape> shape, const Material& material, bool sensor);

		void _waitForBody() const;
	private:
		bool particleCollisionModel = false;
	};
}
