#pragma once

/// \file
/// Declaration of Fresnel functions.

/// Different Fresnel functions.
namespace fluid::renderer::fresnel {
	/// Fresnel function for dielectric materials.
	///
	/// \param cos_in The cosine of the incoming ray angle, i.e., the Y coordinate in tangent space. This parameter
	///               must be positive; if the ray comes from the other direction, simply swap \p eta_in and
	///               \p eta_out.
	/// \param eta_in The index of refraction of the material that the incoming ray is in.
	/// \param eta_out The index of refraction of the material that the refracted ray is in.
	double dielectric(double cos_in, double eta_in_over_out, double eta_out);
	/// \override
	double dielectric(double cos_in, double cos_out, double eta_in, double eta_out);
}
