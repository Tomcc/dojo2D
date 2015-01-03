#pragma once

#include <Dojo.h>

#include <Box2D/Box2D.h>
#include <unordered_set>

#define DLLX __declspec(dllimport)

#include "ConvexDecomposition/b2Polygon.h"

//some STL glue
template< typename T >
using Unique = std::unique_ptr < T > ;

template<typename T>
using Shared = std::shared_ptr < T > ;

template<typename T>
using Weak = std::weak_ptr < T > ;

template<typename T>
using Ref = std::reference_wrapper < T > ;

//C++14 where not available
#ifndef WIN32
template<typename T, typename ...Args>
std::unique_ptr<T> make_unique(Args&& ...args) {
	return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
#else
using std::make_unique;
#endif
using std::make_shared;

namespace Phys {
	typedef uint32_t Group;
	using Vector = Dojo::Vector;
}
