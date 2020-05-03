#include "fluid/renderer/bsdf.h"

/// \file
/// Implementation of different BSDFs.

#include "fluid/math/constants.h"
#include "fluid/math/warping.h"

namespace fluid::renderer {
	namespace bsdfs {
		spectrum lambertian_reflection_brdf::f(vec3d, vec3d) const {
			return reflectance / constants::pi;
		}

		outgoing_ray_sample lambertian_reflection_brdf::sample_f(vec3d norm_in, vec2d random) const {
			outgoing_ray_sample result;
			result.norm_out_direction = warping::unit_hemisphere_from_unit_square_cosine(random);
			result.pdf = warping::pdf_unit_hemisphere_from_unit_square_cosine(result.norm_out_direction);
			result.reflectance = f(norm_in, result.norm_out_direction);
			if (norm_in.y < 0.0) {
				result.norm_out_direction.y = -result.norm_out_direction.y;
			}
			return result;
		}

		double lambertian_reflection_brdf::pdf(vec3d norm_in, vec3d norm_out) const {
			if ((norm_in.y > 0) == (norm_out.y > 0)) {
				norm_out.y = std::abs(norm_out.y);
				return warping::pdf_unit_hemisphere_from_unit_square_cosine(norm_out);
			}
			return 0.0;
		}
	}


	spectrum bsdf::f(vec3d norm_in, vec3d norm_out) const {
		return std::visit(
			[&](const auto &v) {
				return v.f(norm_in, norm_out);
			},
			value
				);
	}

	/// Returns the probability of the given ray being sampled by the material. The input vectors are assumed to
	/// be normalized and in tangent space.
	double bsdf::pdf(vec3d norm_in, vec3d norm_out) const {
		return std::visit(
			[&](const auto &v) {
				return v.pdf(norm_in, norm_out);
			},
			value
				);
	}

	/// Samples an outgoing ray given a incoming ray and a random sample inside a unit square.
	bsdfs::outgoing_ray_sample bsdf::sample_f(vec3d norm_in, vec2d random) const {
		return std::visit(
			[&](const auto &v) {
				return v.sample_f(norm_in, random);
			},
			value
				);
	}
}
