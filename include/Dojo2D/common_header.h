#pragma once

#include <Dojo.h>

#include <Box2D/Box2D.h>
#include <unordered_set>
#include <mutex>

namespace Phys {

	struct Group : public Dojo::PseudoEnumClass<uint16_t> {
		template<typename T>
		Group(T raw) : PseudoEnumClass(raw) {}
	};

	//some reexports
	using Vector = Dojo::Vector;
	using Radians = Dojo::Radians;
	
	template<typename T>
	using optional_ref = Dojo::optional_ref<T>;

	typedef std::lock_guard<std::mutex> ScopedLock;
}
