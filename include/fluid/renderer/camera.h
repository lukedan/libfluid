#pragma once

/// \file
/// Implementation of the basic camera.

#include "../math/vec.h"
#include "common.h"

namespace fluid::renderer {
	/// A basic pinhole camera.
	struct camera {
		vec3d
			position, ///< The position of the camera.
			norm_forward, ///< Normalized forward direction. This corresponds to the center of the screen.
			half_horizontal, ///< Half of the screen's width. Use with \ref norm_forward.
			half_vertical; ///< Half of the screen's height, pointing downwards. Use with \ref norm_forward.

		/// Creates a camera from the given parameters.
		static camera from_parameters(vec3d pos, vec3d ref, vec3d up, double fovy_radians, double width_over_height);

		/// Returns the ray that corresponds to the given screen position. The direction of the ray is *not*
		/// normalized.
		ray get_ray(vec2d screen_pos) const;
	};
}
