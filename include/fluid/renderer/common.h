#pragma once

/// \file
/// Common functions and data structures.

#include <cstddef>
#include <algorithm>
#include <filesystem>
#include <fstream>

#include "../math/vec.h"
#include "../math/mat.h"
#include "../data_structures/grid.h"

namespace fluid::renderer {
	/// A ray.
	struct ray {
		vec3d
			origin, ///< The origin of the ray.
			direction; ///< The direction of the ray. This is not necessarily normalized.
	};


	/// Modulation for basic scalar types by multiplication.
	template <typename T> inline T modulate(const T &lhs, const T &rhs) {
		return lhs * rhs;
	}


	/// Stores an image.
	template <typename Pixel> struct image {
		/// Default constructor.
		image() = default;
		/// Initializes this image with the given size.
		explicit image(vec2s size) : pixels(size) {
		}

		/// Samples this image given the UV coordinates.
		Pixel sample(vec2d uv) const {
			// TODO wrapping modes
			return sample_unit(vec2d(uv.x - std::floor(uv.x), uv.y - std::floor(uv.y)));
		}
		/// Samples this image given the UV coordinates. The coordinates are assumed to be within [0, 1];
		Pixel sample_unit(vec2d uv) const {
			uv = vec_ops::memberwise::mul(uv, vec2d(pixels.get_size())) + vec2d(0.5, 0.5);
			vec2d fract;
			vec2s pos_tl, pos_br;
			vec_ops::for_each(
				[](double uv, double &f, std::size_t &ps, std::size_t &pg, std::size_t size) {
					double ipart;
					f = std::modf(uv, &ipart);
					pg = static_cast<std::size_t>(ipart);
					ps = std::max<std::size_t>(ps, 1) - 1;
					pg = std::min(size - 1, pg);
				},
				uv, fract, pos_tl, pos_br, pixels.get_size()
					);
			Pixel
				pix_tl = pixels(pos_tl.x, pos_tl.y),
				pix_tr = pixels(pos_br.x, pos_tl.y),
				pix_bl = pixels(pos_tl.x, pos_br.y),
				pix_br = pixels(pos_br.x, pos_br.y);
			return bilerp(pix_tl, pix_tr, pix_bl, pix_br, fract.y, fract.x);
		}

		/// Saves this image.
		template <typename ToRGB> void save_ppm(std::filesystem::path p, ToRGB &&torgb) const {
			std::ofstream fout(p);
			fout << "P3\n" << pixels.get_size().x << " " << pixels.get_size().y << "\n255\n";
			for (std::size_t y = 0; y < pixels.get_size().y; ++y) {
				for (std::size_t x = 0; x < pixels.get_size().x; ++x) {
					vec3<std::uint8_t> rgb = torgb(pixels(x, y));
					fout <<
						static_cast<int>(rgb.x) << " " <<
						static_cast<int>(rgb.y) << " " <<
						static_cast<int>(rgb.z) << "\n";
				}
			}
		}

		/// Returns the aspect ration of this image, i.e., width over height.
		double aspect_ratio() const {
			return
				static_cast<double>(pixels.get_size().x) /
				static_cast<double>(pixels.get_size().y);
		}

		grid2<Pixel> pixels; ///< The pixels of this image.
	};


	/// Returns the axis that produces the largest cross product with the input vector.
	inline vec3d get_cross_product_axis(vec3d vec) {
		vec3d abs(std::abs(vec.x), std::abs(vec.y), std::abs(vec.z));
		// select the axis with the smallest absolute value
		if (abs.y > abs.x) {
			if (abs.z > abs.x) {
				return vec3d(1.0, 0.0, 0.0);
			} else {
				return vec3d(0.0, 0.0, 1.0);
			}
		} else {
			if (abs.z > abs.y) {
				return vec3d(0.0, 1.0, 0.0);
			} else {
				return vec3d(0.0, 0.0, 1.0);
			}
		}
	}
	/// Computes an orthonormal matrix used to convert positions from world space to tangent space for the given
	/// normal vector, with the normal mapped to the Y axis. The normal vector is assumed to be normalized.
	inline rmat3d compute_arbitrary_tangent_space(vec3d normal) {
		vec3d x = vec_ops::cross(normal, get_cross_product_axis(normal)).normalized_unchecked();
		vec3d z = vec_ops::cross(x, normal);
		return rmat3d::from_rows(x, normal, z);
	}
}
