#pragma once

/// \file
/// Definition of a series of BSDF (bidirectional scattering distribution function).

#include <variant>

#include "common.h"
#include "spectrum.h"

namespace fluid::renderer {
	namespace bsdfs {
		/// An outgoing ray sample produced by a material.
		struct outgoing_ray_sample {
			spectrum reflectance; ///< Reflectance.
			vec3d norm_out_direction_tangent; ///< Normalized out ray direction in tangent space.
			double pdf = 0.0; ///< The probability density function.
		};

		/// BRDF of Lambertian reflection.
		struct lambertian_reflection_brdf {
			constexpr static bool double_sided = false; ///< Controls whether or not this material is double-sided.

			/// For lambertian materials, the \p f term is constant.
			spectrum f(vec3d, vec3d) const;
			/// Returns the result of \ref fluid::warping::pdf_unit_hemisphere_from_unit_square_cosine();
			double pdf(vec3d, vec3d) const;
			/// The Lambertian material samples rays using
			/// \ref fluid::warping::unit_hemisphere_from_unit_square_cosine();
			outgoing_ray_sample sample_f(vec3d, vec2d) const;

			/// Returns \p false.
			bool is_delta() const;

			spectrum reflectance; ///< The reflectance of this material.
		};

		/// BRDF of specular reflection.
		struct specular_reflection_brdf {
			/// The \p f term is zero unless the two rays are exactly mirrored, but we don't check for that.
			spectrum f(vec3d, vec3d) const;
			/// The probability density function is a delta function and is not modeled here.
			double pdf(vec3d, vec3d) const;
			/// Simply mirrors the input ray and cancels out the Lambertian term.
			outgoing_ray_sample sample_f(vec3d, vec2d) const;

			/// Returns \p true.
			bool is_delta() const;

			spectrum reflectance; ///< The reflectance of this material.
		};

		/// BSDF of specular transmission.
		struct specular_transmission_bsdf {
			/// The \p f term is zero unless the two rays match exactly, but we don't check for that.
			spectrum f(vec3d, vec3d) const;
			/// The probability density function is a delta function and is not modeled here.
			double pdf(vec3d, vec3d) const;
			/// Based on the Fresnel reflectance, reflects or refracts the incoming ray.
			outgoing_ray_sample sample_f(vec3d, vec2d) const;

			/// Returns \p true.
			bool is_delta() const;

			/// The color used for both reflection and transmission.
			spectrum skin;
			double index_of_refraction = 1.0; ///< The IOR of this material.
		};
	}

	/// A generic BSDF.
	struct bsdf {
		/// Storage for different material types.
		using union_t = std::variant<
			bsdfs::lambertian_reflection_brdf,
			bsdfs::specular_reflection_brdf,
			bsdfs::specular_transmission_bsdf
		>;

		/// Returns the value of the \p f term in the light transport equation. The input vectors are assumed to be
		/// normalized and in tangent space.
		spectrum f(vec3d norm_in, vec3d norm_out) const;
		/// Returns the probability of the given ray being sampled by the material. The input vectors are assumed to
		/// be normalized and in tangent space.
		double pdf(vec3d norm_in, vec3d norm_out) const;
		/// Samples an outgoing ray given a incoming ray and a random sample inside a unit square.
		bsdfs::outgoing_ray_sample sample_f(vec3d norm_in, vec2d random) const;

		/// Returns whether this BSDF contains a delta distribution.
		bool is_delta() const;

		union_t value; ///< The value of this BSDF.
		/// The amount of light emitted from this material. This will be ignored if the \p x component is less than
		/// zero.
		spectrum emission;
	};
}
