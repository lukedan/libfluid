#include "fluid/renderer/path_tracer.h"

/// \file
/// Implementation of the basic path tracer.

#include <random>

namespace fluid::renderer {
	const spectrum spectrum::identity(vec3d(1.0, 1.0, 1.0));


	spectrum path_tracer::incoming_light(const scene &scene, const ray &r) {
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
			spectrum isect_atten = sample.reflectance * (std::abs(sample.norm_out_direction.y) / sample.pdf);

			cur_ray = isect.spawn_ray(sample.norm_out_direction);

			attenuation = modulate(attenuation, isect_atten);
		}
		return result;
	}

	image<spectrum> path_tracer::render(const scene &s, const camera &c, vec2s sz) {
		image<spectrum> result;
		result.pixels = grid2<spectrum>(sz, spectrum());
		std::uniform_real_distribution<double> dist(0.0, 1.0);
		vec2d pos_mul = vec_ops::memberwise::div(vec2d(1.0, 1.0), vec2d(sz));
#pragma omp parallel
		{
			pcg32 thread_rng(random());
#pragma omp for
			for (int y = 0; y < sz.y; ++y) {
				for (std::size_t x = 0; x < sz.x; ++x) {
					spectrum pixel;
					vec2d pix_pos(vec2s(x, y));
					for (std::size_t i = 0; i < 100; ++i) {
						vec2d subpix_pos = pix_pos + vec2d(dist(thread_rng), dist(thread_rng));
						subpix_pos = vec_ops::memberwise::mul(subpix_pos, pos_mul);
						subpix_pos.y = 1.0 - subpix_pos.y;
						ray r = c.get_ray(subpix_pos);
						pixel += incoming_light(s, r);
					}
					result.pixels(x, y) = pixel / 100;
				}
				std::cout << y << "\n";
			}
		}
		return result;
	}
}
