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

		void setRevolute(const Vector& localAnchorA, const Vector& localAnchorB);

		bool isInited() const {
			return mJoint.is_some();
		}

		optional_ref<b2Joint> getB2Joint() {
			return mJoint;
		}

		void _init(World& world);
		void _deinit(World& world);

	protected:
		//prismatic
		Vector mLocalAnchorA, mLocalAnchorB;

		const bool mCollideConnected;
		Body& mBodyA, &mBodyB;
		Type mJointType = Type::NotSet;

		optional_ref<b2Joint> mJoint;

		void _init(World& world, b2JointDef& def);
	};
}

