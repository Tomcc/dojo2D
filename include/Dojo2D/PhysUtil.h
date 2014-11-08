#pragma once

#include "common_header.h"

namespace Phys {
	class Body;

	//make a copy of the b2vec 
	inline Vector asVec(const b2Vec2& v) {
		return Vector(v.x, v.y);
	}

	//transmute the reference to a b2vec2 reference
	inline b2Vec2& asB2Vec(Vector& v) {
		return *(b2Vec2*)(&v);
	}

	inline const b2Vec2& asB2Vec(const Vector& v) {
		return *(b2Vec2*)(&v);
	}

	inline Phys::Body& getBodyForFixture(const b2Fixture* f) {
		//not using a reference because b2 is old style
		DEBUG_ASSERT(f, "the fixture may not be null");
		return *(Phys::Body*)f->GetUserData();
	}

	std::vector<b2PolygonShape> decomposeConvex(const std::vector<Vector>& points);
}



