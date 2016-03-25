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

void Joint::setRevolute(const Vector& localAnchorA, const Vector& localAnchorB) {
	DEBUG_ASSERT(mJointType == Type::NotSet, "The joint is already initialized");

	mLocalAnchorA = localAnchorA;
	mLocalAnchorB = localAnchorB;
	mJointType = Type::Revolute;
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
		def.referenceAngle = 0;

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
