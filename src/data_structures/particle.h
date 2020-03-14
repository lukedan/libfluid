#pragma once

/// \file
/// Definition of marker particles.

#include "math/vec.h"

namespace fluid {
	/// A particle.
	struct particle {
		vec3d
			position, ///< The position of this particle.
			velocity; ///< The velocity of this particle.
		vec3s grid_index; ///< The index of this particle in the grid.
	};
}
