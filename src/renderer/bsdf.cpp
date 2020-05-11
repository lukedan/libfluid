#include "fluid/renderer/bsdf.h"

/// \file
/// Implementation of different BSDFs.

#include "fluid/math/constants.h"
#include "fluid/math/warping.h"
#include "fluid/renderer/fresnel.h"

namespace fluid::renderer {
	namespace bsdfs {
		spectrum lambertian_reflection_brdf::f(vec3d in, vec3d out, transport_mode) const {
			return in.y * out.y > 0.0 ? reflectance / constants::pi : spectrum();
		}

		outgoing_ray_sample lambertian_reflection_brdf::sample_f(
			vec3d norm_in, vec2d random, transport_mode mode
		) const {
			outgoing_ray_sample result;
			result.norm_out_direction_tangent = warping::unit_hemisphere_from_unit_square_cosine(random);
			result.pdf = warping::pdf_unit_hemisphere_from_unit_square_cosine(result.norm_out_direction_tangent);
			if constexpr (double_sided) {
				if (norm_in.y < 0.0) {
					result.norm_out_direction_tangent.y = -result.norm_out_direction_tangent.y;
				}
			}
			result.reflectance = f(norm_in, result.norm_out_direction_tangent, mode);
			return result;
		}

		double lambertian_reflection_brdf::pdf(vec3d norm_in, vec3d norm_out) const {
			if constexpr (double_sided) {
				if ((norm_in.y > 0) == (norm_out.y > 0)) {
					norm_out.y = std::abs(norm_out.y);
					return warping::pdf_unit_hemisphere_from_unit_square_cosine(norm_out);
				}
				return 0.0;
			} else {
				return warping::pdf_unit_hemisphere_from_unit_square_cosine(norm_out);;
			}
		}

		bool lambertian_reflection_brdf::is_delta() const {
			return false;
		}


		spectrum specular_reflection_brdf::f(vec3d, vec3d, transport_mode) const {
			return spectrum();
		}

		outgoing_ray_sample specular_reflection_brdf::sample_f(vec3d norm_in, vec2d random, transport_mode) const {
			outgoing_ray_sample result;
			result.norm_out_direction_tangent.x = -norm_in.x;
			result.norm_out_direction_tangent.y = norm_in.y;
			result.norm_out_direction_tangent.z = -norm_in.z;
			result.pdf = 1.0;
			result.reflectance = reflectance / std::abs(norm_in.y); // cancel out Lambertian term
			return result;
		}

		double specular_reflection_brdf::pdf(vec3d norm_in, vec3d norm_out) const {
			return 0.0;
		}

		bool specular_reflection_brdf::is_delta() const {
			return true;
		}


		spectrum specular_transmission_bsdf::f(vec3d, vec3d, transport_mode) const {
			return spectrum();
		}

		outgoing_ray_sample specular_transmission_bsdf::sample_f(
			vec3d norm_in, vec2d random, transport_mode mode
		) const {
			outgoing_ray_sample result;
			double eta_in = 1.0, eta_out = index_of_refraction, cos_in = norm_in.y, sign = 1.0;
			if (cos_in < 0.0) {
				std::swap(eta_in, eta_out);
				cos_in = -cos_in;
				sign = -1.0;
			}
			double eta = eta_in / eta_out;
			double sin2_out = (1.0 - cos_in * cos_in) * eta * eta;
			if (sin2_out >= 1.0) { // total internal reflection
				result.norm_out_direction_tangent = vec3d(-norm_in.x, norm_in.y, -norm_in.z);
				result.pdf = 1.0;
				result.reflectance = skin / cos_in;
				return result;
			}
			double cos_out = std::sqrt(1.0 - sin2_out);
			double fres = fresnel::dielectric(cos_in, cos_out, eta_in, eta_out);
			if (random.x > fres) { // refraction
				result.norm_out_direction_tangent = -eta * norm_in;
				result.norm_out_direction_tangent.y += (eta * cos_in - cos_out) * sign;
				result.pdf = 1.0 - fres;
				result.reflectance = (1.0 - fres) * skin / cos_out;
				if (mode == transport_mode::radiance) {
					result.reflectance *= eta * eta;
				}
			} else { // reflection
				result.norm_out_direction_tangent = vec3d(-norm_in.x, norm_in.y, -norm_in.z);
				result.pdf = fres;
				result.reflectance = fres * skin / cos_in;
			}
			return result;
		}

		double specular_transmission_bsdf::pdf(vec3d norm_in, vec3d norm_out) const {
			return 0.0;
		}

		bool specular_transmission_bsdf::is_delta() const {
			return true;
		}
	}


	spectrum bsdf::f(vec3d norm_in, vec3d norm_out, transport_mode mode) const {
		return std::visit(
			[&](const auto &v) {
				return v.f(norm_in, norm_out, mode);
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
	bsdfs::outgoing_ray_sample bsdf::sample_f(vec3d norm_in, vec2d random, transport_mode mode) const {
		return std::visit(
			[&](const auto &v) {
				return v.sample_f(norm_in, random, mode);
			},
			value
				);
	}

	bool bsdf::is_delta() const {
		return std::visit(
			[](const auto &v) {
				return v.is_delta();
			},
			value
				);
	}
}
