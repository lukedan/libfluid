#pragma once

/// \file
/// Definition of the bidirectional path tracer.

#include <pcg_random.hpp>

#include "common.h"
#include "scene.h"

namespace fluid::renderer {
	/// The bidirectional path tracer.
	class bidirectional_path_tracer {
	public:
		/// Computes the incoming light along the inverse direction of the given ray.
		spectrum incoming_light(const scene&, const ray&, pcg32&) const;

		std::size_t
			max_camera_bounces = 5, ///< Maximum bounces of rays from the camera.
			max_light_bounces = 5; ///< Maximum bounces of rays from light sources.
		double ray_offset = 1e-6; ///< The offset of rays used for raycasting, visibility testing, etc.
	};
}
