#pragma once

#include <Dojo.h>

#include <Box2D/Box2D.h>
#include <unordered_set>
#include <mutex>

namespace Phys {
	typedef uint16_t Group;
	using Vector = Dojo::Vector;
	using Radians = Dojo::Radians;

	typedef std::lock_guard<std::mutex> ScopedLock;
}
