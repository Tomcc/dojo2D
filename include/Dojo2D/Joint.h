#pragma once

namespace Phys {
	class Body;
	class World;

	class Joint {
	public:
		enum class Type {
			NotSet,
			Revolute,
			Distance
		};

		Joint(Body& A, Body& B, bool collideConnected = false);

		void setRevolute(const Vector& localAnchorA, const Vector& localAnchorB, float motorSpeed = FLT_MAX, float maxMotorTorque = FLT_MAX);
		void setDistance(const Vector& worldAnchorA, const Vector& worldAnchorB, float naturalLenght = 0, float dampingRatio = 0, float frequencyHz = 0);

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
		union Desc {
			struct Revolute {
				Vector mLocalAnchorA, mLocalAnchorB;
				float mMotorSpeed, mMaxMotorTorque;
			} revolute;

			struct Distance {
				Vector worldAnchor[2];
				float dampingRatio, frequencyHz, naturalLenght;
			} distance;

			Desc() {}
		} mDesc;

		//generic

		const bool mCollideConnected;
		Body& mBodyA, &mBodyB;
		Type mJointType = Type::NotSet;

		optional_ref<b2Joint> mJoint;

		void _init(World& world, b2JointDef& def);
	};
}

