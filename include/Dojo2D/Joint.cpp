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
	DEBUG_ASSERT(motorSpeed == FLT_MAX or maxMotorTorque < FLT_MAX, "When providing a motor speed, also provide a max torque");

	mJointType = Type::Revolute;
	mDesc.revolute = { localAnchorA, localAnchorB, motorSpeed, maxMotorTorque };
}

void Phys::Joint::setDistance(const Vector& worldAnchorA, const Vector& worldAnchorB, float naturalLenght, float dampingRatio /*= 0*/, float frequencyHz /*= 0*/, float yScale) {
	DEBUG_ASSERT(mJointType == Type::NotSet, "The joint is already initialized");

	mJointType = Type::Distance;
	mDesc.distance = {
		{ worldAnchorA, worldAnchorB },
		dampingRatio,
		frequencyHz,
		naturalLenght,
		yScale
	};
}

float Phys::Joint::getDistanceJointLength() const {
	DEBUG_ASSERT(mJointType == Type::Distance, "Invalid joint type");
	return static_cast<b2DistanceJoint&>(mJoint.unwrap()).GetLength();
}

void Phys::Joint::setDistanceJointLength(float l) {
	DEBUG_ASSERT(mJointType == Type::Distance, "Invalid joint type");
	DEBUG_ASSERT(l > 0, "Invalid length");

	static_cast<b2DistanceJoint&>(mJoint.unwrap()).SetLength(l);
}

void Joint::_init(World& world, b2JointDef& def) {
	//this is executed on the physics thread
	def.bodyA = &mBodyA.getB2Body().unwrap();
	def.bodyB = &mBodyB.getB2Body().unwrap();
	def.collideConnected = mCollideConnected;

	mBodyA._registerJoint(self);
	mBodyB._registerJoint(self);

	mJoint = *world.getBox2D().CreateJoint(&def);
}

void Joint::_init(World& world) {
	//this is executed on the physics thread
	switch (mJointType)
	{
	case Joint::Type::Revolute: {
		b2RevoluteJointDef def;
		def.localAnchorA = asB2Vec(mDesc.revolute.mLocalAnchorA);
		def.localAnchorB = asB2Vec(mDesc.revolute.mLocalAnchorB);
		def.referenceAngle = mBodyB.getB2Body().unwrap().GetAngle() - mBodyA.getB2Body().unwrap().GetAngle();

		if(mDesc.revolute.mMotorSpeed < FLT_MAX) {
			def.enableMotor = true;
			def.motorSpeed = mDesc.revolute.mMotorSpeed;
			def.maxMotorTorque = mDesc.revolute.mMaxMotorTorque;
		}

		_init(world, def);
		break;
	}
	case Joint::Type::Distance: {
		b2DistanceJointDef def;
		def.Initialize(
			&mBodyA.getB2Body().unwrap(),
			&mBodyB.getB2Body().unwrap(),
			asB2Vec(mDesc.distance.worldAnchor[0]),
			asB2Vec(mDesc.distance.worldAnchor[1])
		);

		//override lenght if needed
		if(mDesc.distance.naturalLenght) {
			def.length = mDesc.distance.naturalLenght;
		}

		def.frequencyHz = mDesc.distance.frequencyHz;
		def.dampingRatio = mDesc.distance.dampingRatio;

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
