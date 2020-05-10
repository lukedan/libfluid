#pragma once

/// \file
/// Definition of spectrum types.

#include <iostream>

#include "fluid/math/vec.h"
#include "fluid/renderer/common.h"

namespace fluid::renderer {
	/// Used to represent a spectrum.
	struct rgb_spectrum {
		/// Default constructor.
		rgb_spectrum() = default;

		const static rgb_spectrum identity; /// A spectrum with value one on all wavelengths.

		/// Converts this spectrum to RGB values.
		vec3d to_rgb() const {
			return _rgb;
		}
		/// Creates a new spectrum from the given RGB values.
		inline static rgb_spectrum from_rgb(vec3d rgb) {
			return rgb_spectrum(rgb);
		}

		/// Debug output.
		friend std::ostream &operator<<(std::ostream &out, const rgb_spectrum &rgb) {
			return out << "rgb(" << rgb._rgb.x << ", " << rgb._rgb.y << ", " << rgb._rgb.z << ")";
		}

		/// In-place addition.
		rgb_spectrum &operator+=(const rgb_spectrum &rhs) {
			_rgb += rhs._rgb;
			return *this;
		}
		/// Addition.
		friend rgb_spectrum operator+(const rgb_spectrum &lhs, const rgb_spectrum &rhs) {
			rgb_spectrum result = lhs;
			return result += rhs;
		}

		/// In-place subtraction.
		rgb_spectrum &operator-=(const rgb_spectrum &rhs) {
			_rgb -= rhs._rgb;
			return *this;
		}
		/// Subtraction.
		friend rgb_spectrum operator-(const rgb_spectrum &lhs, const rgb_spectrum &rhs) {
			rgb_spectrum result = lhs;
			return result -= rhs;
		}

		/// In-place scalar multiplication.
		rgb_spectrum &operator*=(double rhs) {
			_rgb *= rhs;
			return *this;
		}
		/// Scalar multiplication.
		friend rgb_spectrum operator*(rgb_spectrum lhs, double rhs) {
			return lhs *= rhs;
		}
		/// Scalar multiplication.
		friend rgb_spectrum operator*(double lhs, rgb_spectrum rhs) {
			return rhs *= lhs;
		}

		/// In-place scalar division.
		rgb_spectrum &operator/=(double rhs) {
			_rgb /= rhs;
			return *this;
		}
		/// Scalar division.
		friend rgb_spectrum operator/(rgb_spectrum lhs, double rhs) {
			return lhs /= rhs;
		}

		/// Tests if this specturm is almost black.
		bool near_zero(double threshold = 1e-6) const {
			return _rgb.x < threshold && _rgb.y < threshold && _rgb.z < threshold;
		}

		/// Modulate spectrum by memberwise multiplication.
		friend inline rgb_spectrum modulate(const rgb_spectrum &lhs, const rgb_spectrum &rhs) {
			return rgb_spectrum(vec_ops::memberwise::mul(lhs._rgb, rhs._rgb));
		}
	private:
		/// Directly initializes this spectrum.
		explicit rgb_spectrum(vec3d value) : _rgb(value) {
		}

		vec3d _rgb; ///< The RGB values.
	};

	using spectrum = rgb_spectrum; ///< The default spectrum type.
}
