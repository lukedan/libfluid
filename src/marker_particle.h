#pragma once

/// \file
/// Definition of marker particles.

#include "math/vec.h"

namespace fluid {
	/// A marker particle.
	struct marker_particle {
		vec3d position; ///< The position of this particle.
	};
}
