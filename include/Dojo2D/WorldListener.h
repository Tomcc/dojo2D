#pragma once

#include "common_header.h"

namespace Phys {
	class WorldListener {
	public:
		virtual ~WorldListener() {
		}

		virtual void onPostSimulationStep() = 0;
	};
}
