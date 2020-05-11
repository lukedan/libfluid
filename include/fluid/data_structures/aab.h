#pragma once

/// \file
/// Axis-aligned boxes.

#include <cstddef>

namespace fluid {
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
		[[nodiscard]] T get_volume() const {
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

		/// Returns the center of this box.
		[[nodiscard]] vec_type get_center() const {
			return (min + max) / static_cast<T>(2);
		}

		vec_type
			min, ///< The minimum corner.
			max; ///< The maximum corner.
	};

	template <typename T> using aab2 = aab<2, T>; ///< 2D axis-aligned bounding boxes.
	using aab2d = aab2<double>; ///< 2D double axis-aligned bounding boxes.

	template <typename T> using aab3 = aab<3, T>; ///< 3D axis-aligned bounding boxes.
	using aab3d = aab3<double>; ///< 3D double axis-aligned bounding boxes.
}
