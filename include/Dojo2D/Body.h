#pragma once

#include "common_header.h"

namespace Phys {

	class Material;
	class World;
	class BodyPart;
	class Body;
	class Joint;

	class CollisionListener {
	public:
		virtual void onCollision(Phys::BodyPart& me, Phys::BodyPart& other, float force, const Vector& point) {}
		virtual void onSensorCollision(Body& other, b2Fixture& sensor) {};
	};

	class Body : public Dojo::Component	{
	public:
		static const int ID = 1;

		CollisionListener* collisionListener = nullptr;

		Body(Dojo::Object& object, World& world, Group group, bool staticShape = false, bool inactive = false);

		~Body();

		Body(const Body&) = delete;
		Body(Body&&) = delete;

		Body& operator=(const Body&) = delete;
		Body& operator=(Body&&) = delete;

		BodyPart& addPolyShape(const Material& material, const Vector* points, size_t count, Group group = Group::None, bool sensor = false);
		BodyPart& addPolyShape(const Material& material, const std::vector<Vector>& points, Group group = Group::None, bool sensor = false);
		BodyPart& addBoxShape(const Material& material, const Vector& dimensions, const Vector& center = Vector::Zero, Group group = Group::None, bool sensor = false);
		BodyPart& addCircleShape(const Material& material, float radius, const Vector& center = Vector::Zero, Group group = Group::None, bool sensor = false);
		BodyPart& addCapsuleShape(const Material& material, const Vector& dimensions, const Vector& center = Vector::Zero, Group group = Group::None, bool sensor = false);

		void removeShape(BodyPart& part);

		///Removes physical behaviors from this object
		void destroyPhysics();

		void enableParticleCollisions() {
			mParticleCollisionModel = true;
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

		float getMass() const;
		float getWeight() const;

		Vector getLocalPoint(const Vector& worldPosition) const;
		Vector getWorldPoint(const Vector& localPosition) const;

		Vector getVelocity() const;
		Vector getVelocityAtLocalPoint(const Vector& localPoint) const;
		float getAngularVelocity() const;

		Vector getPosition() const;

		float getLinearDamping() const;
		float getAngularDamping() const;
		
		void setDamping(float linear, float angular);

		void setTransform(const Vector& position, Radians angle);

		void setPushable(bool p) {
			mPushable = p;
		}

		Group getDefaultGroup() const {
			return mDefaultGroup;
		}

		optional_ref<b2Body> getB2Body() const {
			return mBody;
		}

		bool isStatic() const {
			return mStaticShape;
		}

		bool isParticle() const {
			return mParticleCollisionModel;
		}

		void onSimulationPaused();

		void updateObject();

		World& getWorld() const {
			return mWorld;
		}

		const Dojo::SmallSet<Shared<BodyPart>>& getParts() const {
			return mParts;
		}

		float getMinimumDistanceTo(const Vector& pos) const;

		void onAttach() override;
		void onDestroy(Unique<Component> myself) override;
		void onDispose() override;

		bool isPushable() const;

		const Dojo::SmallSet<Joint*> getJoints() const {
			return mJoints;
		}

		void _registerJoint(Joint& joint);
		void _removeJoint(Joint& joint);

	protected:
		World& mWorld;
		bool mPushable = true;

		optional_ref<b2Body> mBody;
		Group mDefaultGroup = Group::None;
		bool mStaticShape = false;
		bool mAutoActivate;

		Dojo::SmallSet<Shared<BodyPart>> mParts;
		Dojo::SmallSet<Joint*> mJoints;

		BodyPart& _addShape(Shared<b2Shape> shape, const Material& material, Group group, bool sensor);

		b2Body& _waitForBody() const;
	private:
		bool mParticleCollisionModel = false;
	};
}
