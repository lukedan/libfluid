#include "fluid/renderer/fresnel.h"

/// \file
/// Implementations of different Fresnel functions.

#include <cmath>
#include <algorithm>

namespace fluid::renderer::fresnel {
	double dielectric(double cos_in, double eta_in, double eta_out) {
		double eta = eta_in / eta_out;
		double sin2_out = (1.0 - cos_in * cos_in) * eta * eta;
		double one_minus_squared_sin = 1.0 - sin2_out;
		if (one_minus_squared_sin <= 0.0) { // total internal reflection
			return 1.0;
		}
		double cos_out = std::sqrt(std::max(0.0, one_minus_squared_sin));
		return dielectric(cos_in, cos_out, eta_in, eta_out);
	}

	double dielectric(double cos_in, double cos_out, double eta_in, double eta_out) {
		double r_parallel =
			(eta_out * cos_in - eta_in * cos_out) /
			(eta_out * cos_in + eta_in * cos_out);
		double r_perpendicular =
			(eta_in * cos_in - eta_out * cos_out) /
			(eta_in * cos_in + eta_out * cos_out);
		return 0.5 * (r_parallel * r_parallel + r_perpendicular * r_perpendicular);
	}
}
