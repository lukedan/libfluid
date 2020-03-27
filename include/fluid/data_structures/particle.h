#pragma once

/// \file
/// Definition of marker particles.

#include "fluid/math/vec.h"

namespace fluid {
	/// A particle.
	struct particle {
		vec3d
			position, ///< The position of this particle.
			velocity, ///< The velocity of this particle.
			cx, ///< The c vector used in APIC.
			cy, ///< The c vector used in APIC.
			cz; ///< The c vector used in APIC.
		vec3s grid_index; ///< The index of this particle in the grid.
	};
}
