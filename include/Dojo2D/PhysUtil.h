#pragma once

#include "common_header.h"

namespace Phys {
	class Body;

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

	inline Phys::Body& getBodyForFixture(const b2Fixture& f) {
		return *(Phys::Body*)f.GetUserData();
	}

	inline Phys::Body& getBodyForFixture(const b2Fixture* f) {
		//not using a reference because b2 is old style
		DEBUG_ASSERT(f, "the fixture may not be null");
		return getBodyForFixture(*f);
	}

	std::vector<b2PolygonShape> decomposeConvex(const std::vector<Dojo::Vector>& points);
}



