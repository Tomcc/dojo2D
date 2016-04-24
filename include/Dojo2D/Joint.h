#pragma once

namespace Phys {
	class Body;
	class World;

	class Joint {
	public:
		enum class Type {
			NotSet,
			Revolute
		};

		Joint(Body& A, Body& B, bool collideConnected = false);

		void setRevolute(const Vector& localAnchorA, const Vector& localAnchorB, float motorSpeed = FLT_MAX, float maxMotorTorque = FLT_MAX);

		bool isInited() const {
			return mJoint.is_some();
		}

		optional_ref<b2Joint> getB2Joint() {
			return mJoint;
		}

		void _init(World& world);
		void _deinit(World& world);

	protected:
		//revolute
		Vector mLocalAnchorA, mLocalAnchorB;
		float mMotorSpeed, mMaxMotorTorque;

		//generic

		const bool mCollideConnected;
		Body& mBodyA, &mBodyB;
		Type mJointType = Type::NotSet;

		optional_ref<b2Joint> mJoint;

		void _init(World& world, b2JointDef& def);
	};
}

