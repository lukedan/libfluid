#include "fluid/renderer/path_tracer.h"

/// \file
/// Implementation of the basic path tracer.

#include <random>

namespace fluid::renderer {
	const spectrum spectrum::identity(vec3d(1.0, 1.0, 1.0));


	spectrum path_tracer::incoming_light(const scene &scene, const ray &r, pcg32 &random) const {
		spectrum result;
		ray cur_ray = r;
		cur_ray.direction = cur_ray.direction.normalized_unchecked();
		spectrum attenuation = spectrum::identity;
		std::uniform_real_distribution<double> dist(0.0, 1.0);
		for (std::size_t i = 0; i < max_bounces; ++i) {
			auto [prim, res, isect] = scene.ray_cast(cur_ray);
			if (prim == nullptr) {
				break;
			}

			// direct lighting
			result += modulate(attenuation, isect.surface_bsdf.emission);

			// sample outgoing ray
			vec3d incoming_direction = isect.tangent * -cur_ray.direction;
			bsdfs::outgoing_ray_sample sample =
				isect.surface_bsdf.sample_f(incoming_direction, vec2d(dist(random), dist(random)));
			spectrum isect_atten = sample.reflectance * (std::abs(sample.norm_out_direction_tangent.y) / sample.pdf);

			cur_ray = isect.spawn_ray(sample.norm_out_direction_tangent);

			attenuation = modulate(attenuation, isect_atten);
		}
		return result;
	}
}
