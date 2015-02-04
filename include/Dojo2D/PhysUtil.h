#pragma once

#include "common_header.h"

namespace Phys {
	class Body;
	class Material;

	//make a copy of the b2vec 
	inline Dojo::Vector asVec(const b2Vec2& v) {
		return{ v.x, v.y };
	}

	//transmute the reference to a b2vec2 reference
	inline b2Vec2& asB2Vec(Dojo::Vector& v) {
		return *(b2Vec2*)(&v);
	}

	inline const b2Vec2& asB2Vec(const Dojo::Vector& v) {
		return *(b2Vec2*)(&v);
	}

	inline Phys::Body& getBodyForFixture(const b2Fixture& fixture) {
		DEBUG_ASSERT(fixture.GetBody()->GetUserData(), "Malformed fixture without an owner Body");
		return *(Phys::Body*)fixture.GetBody()->GetUserData();
	}

	inline Phys::Body& getBodyForFixture(const b2Fixture* fixture) {
		//not using a reference because b2 is old style
		DEBUG_ASSERT(fixture, "the fixture may not be null");
		return getBodyForFixture(*fixture);
	}

	inline const Material& getMaterialForFixture(const b2Fixture& fixture) {
		return *(const Material*)fixture.GetUserData();
	}

	inline const Material& getMaterialForFixture(const b2Fixture* fixture) {
		//not using a reference because b2 is old style
		DEBUG_ASSERT(fixture, "the fixture may not be null");
		return *(const Material*)fixture->GetUserData();
	}

	std::vector<b2PolygonShape> decomposeConvex(const std::vector<Dojo::Vector>& points);
}



