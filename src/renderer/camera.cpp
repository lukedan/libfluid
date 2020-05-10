#include "fluid/renderer/camera.h"

/// \file
/// Implementation of the camera.

#include "fluid/renderer/common.h"

namespace fluid::renderer {
	camera camera::from_parameters(vec3d pos, vec3d ref, vec3d up, double fovy_radians, double width_over_height) {
		camera result;
		result.position = pos;
		result.norm_forward = (ref - pos).normalized_unchecked();

		double tan_half = std::tan(0.5 * fovy_radians);
		result.half_horizontal = vec_ops::cross(result.norm_forward, up)
			.normalized_checked()
			.value_or(get_cross_product_axis(result.norm_forward));
		result.half_vertical = vec_ops::cross(result.norm_forward, result.half_horizontal);

		result.half_horizontal *= tan_half * width_over_height;
		result.half_vertical *= tan_half;

		return result;
	}

	ray camera::get_ray(vec2d screen_pos) const {
		screen_pos = screen_pos * 2.0 - vec2d(1.0, 1.0);
		ray result;
		result.origin = position;
		result.direction = norm_forward + screen_pos.x * half_horizontal + screen_pos.y * half_vertical;
		return result;
	}
}
