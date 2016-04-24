#include "Joint.h"

#include "PhysUtil.h"
#include "Body.h"
#include "World.h"

using namespace Phys;

Joint::Joint(Body& A, Body& B, bool collideConnected /*= false*/) :
mBodyA(A),
mBodyB(B),
mCollideConnected(collideConnected) {

}

void Phys::Joint::setRevolute(const Vector& localAnchorA, const Vector& localAnchorB, float motorSpeed /*= FLT_MAX*/, float maxMotorTorque /*= FLT_MAX*/) {
	DEBUG_ASSERT(mJointType == Type::NotSet, "The joint is already initialized");
	DEBUG_ASSERT(motorSpeed == FLT_MAX || maxMotorTorque < FLT_MAX, "When providing a motor speed, also provide a max torque");

	mLocalAnchorA = localAnchorA;
	mLocalAnchorB = localAnchorB;
	mJointType = Type::Revolute;
	mMotorSpeed = motorSpeed;
	mMaxMotorTorque = maxMotorTorque;
}

void Joint::_init(World& world, b2JointDef& def) {
	//this is executed on the physics thread
	def.bodyA = &mBodyA.getB2Body().unwrap();
	def.bodyB = &mBodyB.getB2Body().unwrap();
	def.collideConnected = mCollideConnected;

	mJoint = *world.getBox2D().CreateJoint(&def);
}

void Joint::_init(World& world) {
	//this is executed on the physics thread
	switch (mJointType)
	{
	case Joint::Type::Revolute: {
		b2RevoluteJointDef def;
		def.localAnchorA = asB2Vec(mLocalAnchorA);
		def.localAnchorB = asB2Vec(mLocalAnchorB);
		def.referenceAngle = mBodyB.getB2Body().unwrap().GetAngle() - mBodyA.getB2Body().unwrap().GetAngle();

		if(mMotorSpeed < FLT_MAX) {
			def.enableMotor = true;
			def.motorSpeed = mMotorSpeed;
			def.maxMotorTorque = mMaxMotorTorque;
		}

		_init(world, def);
		break;
	}
	default:
		FAIL("Invalid joint. Not initialized?");
	}
}

void Joint::_deinit(World& world) {
	mBodyA._removeJoint(self);
	mBodyB._removeJoint(self);

	//this is executed on the physics thread
	world.getBox2D().DestroyJoint(&mJoint.unwrap());

	mJoint = {};
	mJointType = Type::NotSet;
}
