#pragma once

#include "common_header.h"

namespace Phys {
	class Body;
	class BodyPart;

	//make a copy of the b2vec 
	inline Dojo::Vector asVec(const b2Vec2& v) {
		return{ v.x, v.y };
	}

	//transmute the reference to a b2vec2 reference
	inline const b2Vec2& asB2Vec(const Dojo::Vector& v) {
		return *(b2Vec2*)(&v);
	}

	inline Phys::Body& getBodyForFixture(const b2Fixture& fixture) {
		DEBUG_ASSERT(fixture.getBody()->GetUserData(), "Malformed fixture without an owner Body");
		return *(Phys::Body*)fixture.getBody()->GetUserData();
	}

	inline Phys::Body& getBodyForFixture(const b2Fixture* fixture) {
		//not using a reference because b2 is old style
		DEBUG_ASSERT(fixture, "the fixture may not be null");
		return getBodyForFixture(*fixture);
	}

	inline const BodyPart& getPartForFixture(const b2Fixture& fixture) {
		return *(const BodyPart*)fixture.GetUserData();
	}

	inline const BodyPart& getPartForFixture(const b2Fixture* fixture) {
		//not using a reference because b2 is old style
		DEBUG_ASSERT(fixture, "the fixture may not be null");
		return *(const BodyPart*)fixture->GetUserData();
	}

	inline BodyPart& getPartForFixture(b2Fixture* fixture) {
		//not using a reference because b2 is old style
		DEBUG_ASSERT(fixture, "the fixture may not be null");
		return *(BodyPart*)fixture->GetUserData();
	}

	std::vector<b2PolygonShape> decomposeConvex(const std::vector<Dojo::Vector>& points);
}



