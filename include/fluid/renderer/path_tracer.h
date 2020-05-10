#pragma once

/// \file
/// Basic path tracer.

#include <pcg_random.hpp>

#include "../math/mat.h"
#include "common.h"
#include "bsdf.h"
#include "scene.h"
#include "camera.h"

namespace fluid::renderer {
	/// Basic path tracer.
	class path_tracer {
	public:
		/// Computes the incoming light along the inverse direction of the given ray.
		spectrum incoming_light(const scene&, const ray&, pcg32&) const;

		std::size_t max_bounces = 5; ///< The maximum number of ray bounces.
	};
}
