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
	/// Axis-aligned boxes.
	template <std::size_t Dim, typename T> struct aab {
		using vec_type = vec<Dim, T>; ///< The vector type.

		/// Default constructor.
		aab() = default;
		/// Initializes all fields of this struct.
		aab(vec_type minp, vec_type maxp) : min(minp), max(maxp) {
		}

		/// Returns the \ref aab that contains all given points.
		template <typename ...Args> [[nodiscard]] inline static aab containing(
			const vec_type &first, const Args &...args
		) {
			aab result(first, first);
			(result.make_contain(args), ...);
			return result;
		}
		/// Returns the \ref aab that contains all given points.
		template <typename It> [[nodiscard]] inline static aab containing_dynamic(It &&begin, It &&end) {
			if (begin == end) {
				return aab();
			}
			aab result;
			result.min = result.max = *begin;
			std::decay_t<It> it = begin;
			for (++it; it != end; ++it) {
				result.make_contain(*it);
			}
			return result;
		}

		/// Adjusts \ref min and \ref max to make this \ref aab contain the given point.
		void make_contain(const vec_type &vec) {
			vec_ops::for_each(
				[](T &minv, T &maxv, const T &cur) {
					minv = std::min(minv, cur);
					maxv = std::max(maxv, cur);
				},
				min, max, vec
					);
		}

		/// Returns whether the two bounding boxes intersect. If the bounding boxes use integer coordinates, then
		/// them simply "touching" will count as intersecting.
		[[nodiscard]] inline static bool intersects(const aab &lhs, const aab &rhs) {
			bool result = true;
			vec_ops::for_each(
				[&result](const T &min1, const T &max1, const T &min2, const T &max2) {
					if (max1 < min2 || min1 > max2) {
						result = false;
					}
				},
				lhs.min, lhs.max, rhs.min, rhs.max
					);
			return result;
		}
		/// Returns the intersection of the given two bounding boxes. If the bounding boxes do not intersect, a
		/// bounding box with negative volume will be returned.
		[[nodiscard]] inline static aab intersection(const aab &lhs, const aab &rhs) {
			aab result;
			result.min = vec_ops::apply<vec_type>(
				static_cast<const T & (*)(const T&, const T&)>(std::max), lhs.min, rhs.min
			);
			result.max = vec_ops::apply<vec_type>(
				static_cast<const T & (*)(const T&, const T&)>(std::min), lhs.max, rhs.max
			);
			return result;
		}
		/// Returns the \ref aab that bounds the given two bounding boxes.
		[[nodiscard]] inline static aab bounding(const aab &lhs, const aab &rhs) {
			aab result;
			result.min = vec_ops::apply<vec_type>(
				static_cast<const T & (*)(const T&, const T&)>(std::min), lhs.min, rhs.min
			);
			result.max = vec_ops::apply<vec_type>(
				static_cast<const T & (*)(const T&, const T&)>(std::max), lhs.max, rhs.max
			);
			return result;
		}

		/// Computes the size of this box.
		[[nodiscard]] vec_type get_size() const {
			return max - min;
		}
		/// Computes the area (in two dimensions) or volume (in three or more dimensions) of this box.
		T get_volume() const {
			vec_type size = get_size();
			T result = static_cast<T>(1);
			vec_ops::for_each(
				[&result](const T &sz) {
					result *= sz;
				},
				size
					);
			return result;
		}

		vec_type
			min, ///< The minimum corner.
			max; ///< The maximum corner.
	};

	template <typename T> using aab2 = aab<2, T>; ///< 2D axis-aligned bounding boxes.
	using aab2d = aab2<double>; ///< 2D double axis-aligned bounding boxes.

	template <typename T> using aab3 = aab<3, T>; ///< 3D axis-aligned bounding boxes.
	using aab3d = aab3<double>; ///< 3D double axis-aligned bounding boxes.


	/// A ray.
	struct ray {
		vec3d
			origin, ///< The origin of the ray.
			direction; ///< The direction of the ray. This is not necessarily normalized.
	};


	/// Used to represent a spectrum.
	struct spectrum {
		/// Default constructor.
		spectrum() = default;
		/// Directly initializes this spectrum.
		explicit spectrum(vec3d value) : rgb(value) {
		}

		const static spectrum identity; /// A spectrum with value one on all wavelengths.

		/// In-place addition.
		spectrum &operator+=(const spectrum &rhs) {
			rgb += rhs.rgb;
			return *this;
		}
		/// Addition.
		friend spectrum operator+(const spectrum &lhs, const spectrum &rhs) {
			spectrum result = lhs;
			return result += rhs;
		}

		/// In-place subtraction.
		spectrum &operator-=(const spectrum &rhs) {
			rgb -= rhs.rgb;
			return *this;
		}
		/// Subtraction.
		friend spectrum operator-(const spectrum &lhs, const spectrum &rhs) {
			spectrum result = lhs;
			return result -= rhs;
		}

		/// In-place scalar multiplication.
		spectrum &operator*=(double rhs) {
			rgb *= rhs;
			return *this;
		}
		/// Scalar multiplication.
		friend spectrum operator*(spectrum lhs, double rhs) {
			return lhs *= rhs;
		}
		/// Scalar multiplication.
		friend spectrum operator*(double lhs, spectrum rhs) {
			return rhs *= lhs;
		}

		/// In-place scalar division.
		spectrum &operator/=(double rhs) {
			rgb /= rhs;
			return *this;
		}
		/// Scalar division.
		friend spectrum operator/(spectrum lhs, double rhs) {
			return lhs /= rhs;
		}

		vec3d rgb; ///< The RGB values.
	};

	/// Modulation for basic scalar types by multiplication.
	template <typename T> inline T modulate(const T &lhs, const T &rhs) {
		return lhs * rhs;
	}
	/// Modulate spectrum by memberwise multiplication.
	inline spectrum modulate(const spectrum &lhs, const spectrum &rhs) {
		return spectrum(vec_ops::memberwise::mul(lhs.rgb, rhs.rgb));
	}


	/// Stores an image.
	template <typename Pixel> struct image {
		/// Samples this image given the UV coordinates.
		Pixel sample(vec2d uv) const {
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
		template <typename SavePixel> void save_ppm(std::filesystem::path p, SavePixel &&sp) const {
			std::ofstream fout(p);
			fout << "P3\n" << pixels.get_size().x << " " << pixels.get_size().y << "\n255\n";
			for (std::size_t y = 0; y < pixels.get_size().y; ++y) {
				for (std::size_t x = 0; x < pixels.get_size().x; ++x) {
					sp(fout, pixels(x, y));
					fout << "\n";
				}
			}
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
		return rmat3d(rmat3d::storage_type(x, normal, z));
	}
}
