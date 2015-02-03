#pragma once

#include "common_header.h"

namespace Phys {
	
	class Material;
	class World;

	class Body
	{
	public:
		Body(World& world);

		virtual ~Body();

		b2Fixture& addPolyShape(const Material& material, const b2Vec2* points, int count, bool sensor = false);
		b2Fixture& addPolyShape(const Material& material, const std::vector<b2Vec2>& points, bool sensor = false);
		b2Fixture& addBoxShape(const Material& material, const Vector& dimensions, const Vector& center = Vector::ZERO, bool sensor = false);
		b2Fixture& addCircleShape(const Material& material, float radius, const Vector& center = Vector::ZERO, bool sensor = false);
		b2Fixture& addCapsuleShape(const Material& material, const Vector& dimensions, const Vector& center = Vector::ZERO, bool sensor = false);

		void initPhysics(Dojo::Renderable& graphics, Group group, bool staticShape = false);
		void initPhysics(Dojo::Object& level, Group group, bool staticShape = false);

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
		void forceRotation(float angle);

		virtual float getMass() const;

		Vector getPosition() const;

		Vector getLocalPoint(const Vector& worldPosition) const;
		Vector getWorldPoint(const Vector& localPosition) const;

		Vector getVelocity() const;
		Vector getVelocityAtLocalPoint(const Vector& localPoint) const;

		void setTransform(const Vector& position, float angle);

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

		virtual void onCollision(Body& other, float force, const Vector& point) {}
		virtual void onSensorCollision(Body& other, b2Fixture& sensor) {};

		void updateGraphics();

		const World& getWorld() const {
			return world;
		}

	protected:

		World& world;
		Dojo::Renderable* graphics = nullptr;

		b2Body* body;
		Group group;
		bool staticShape = false;

		b2Fixture& _addShape(b2Shape& shape, const Material& material, bool sensor);

		void _init(Dojo::Object& box2D, Dojo::Renderable* graphics, Group group, bool staticShape = false);
	private:
		bool particleCollisionModel = false;
	};
}
