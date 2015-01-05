#pragma once

#include <Dojo.h>

#include <Box2D/Box2D.h>
#include <unordered_set>

#define DLLX __declspec(dllimport)

#include "ConvexDecomposition/b2Polygon.h"

namespace Phys {
	typedef uint32_t Group;
	using Vector = Dojo::Vector;
}
