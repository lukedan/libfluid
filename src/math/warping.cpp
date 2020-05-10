#include "fluid/math/warping.h"

/// \file
/// Implementation of warping functions.

#include <algorithm>

#include "fluid/math/constants.h"

namespace fluid::warping {
	vec2d unit_disk_from_unit_square(vec2d square) {
		double r = std::sqrt(square.x), angle = square.y * 2.0 * constants::pi;
		return vec2d(r * std::cos(angle), r * std::sin(angle));
	}

	vec2d unit_disk_from_unit_square_concentric(vec2d square) {
		vec2d p = 2.0 * square - vec2d(1.0, 1.0);
		double radius, phi;
		if (p.x > -p.y) {
			if (p.x > p.y) {
				radius = p.x;
				phi = p.y / p.x;
			} else {
				radius = p.y;
				phi = 2.0 - p.x / p.y;
			}
		} else {
			if (p.x < p.y) {
				radius = -p.x;
				phi = 4.0 + p.y / p.x;
			} else {
				radius = -p.y;
				phi = 6.0 - p.x / p.y;
			}
		}
		phi = phi * constants::pi / 4.0;
		return vec2d(radius * std::cos(phi), radius * std::sin(phi));
	}

	double pdf_unit_disk_from_unit_square() {
		return 1.0 / constants::pi;
	}


	vec3d unit_sphere_from_unit_square(vec2d square) {
		double cos_phi = square.x * 2.0 - 1.0, sin_phi = std::sqrt(1.0 - cos_phi * cos_phi);
		double theta = square.y * 2.0f * constants::pi;
		return vec3d(sin_phi * std::cos(theta), sin_phi * std::sin(theta), cos_phi);
	}

	double pdf_unit_sphere_from_unit_square() {
		return 1.0 / (4.0 * constants::pi);
	}


	vec3d unit_hemisphere_from_unit_square(vec2d square) {
		double cosphi = square.x, sinphi = std::sqrt(1.0 - cosphi * cosphi);
		double theta = square.y * 2.0 * constants::pi;
		return vec3d(sinphi * std::cos(theta), cosphi, sinphi * std::sin(theta));
	}

	double pdf_unit_hemisphere_from_unit_square() {
		return 2.0 / (4.0 * constants::pi);
	}


	vec3d unit_hemisphere_from_unit_square_cosine(vec2d square) {
		vec2d result = unit_disk_from_unit_square_concentric(square);
		double y = std::sqrt(std::max(0.0, 1.0 - result.squared_length()));
		return vec3d(result.x, y, result.y);
	}

	double pdf_unit_hemisphere_from_unit_square_cosine(vec3d sample) {
		return sample.y / constants::pi;
	}
}
