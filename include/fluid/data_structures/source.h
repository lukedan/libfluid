#pragma once

/// \file
/// Definition of fluid sources.

#include <vector>

#include "math/vec.h"

namespace fluid {
	/// A fluid source.
	class source {
	public:
		std::vector<vec3s> cells; ///< The cells where fluid particles will be spawned.
		vec3d velocity; ///< The velocity of spawned particles.
		std::size_t target_density_cubic_root = 2; ///< Cubic root of target seeding density.
		bool
			active = true, ///< Whether or not this source is active.
			/// If \p true, will cause the simulator to override the velocities of all particles in \ref cells with
			/// \ref velocity.
			coerce_velocity = false;
	};
}
