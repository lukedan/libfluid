#pragma once

/// \file
/// Common functions and data structures.

#include <cstddef>
#include <algorithm>

#include "../math/vec.h"

namespace fluid::renderer {
	/// Axis-aligned bounding boxes.
	template <std::size_t Dim, typename T> struct aabb {
		using vec_type = vec<Dim, T>; ///< The vector type.

		/// Default constructor.
		aabb() = default;
		/// Initializes all fields of this struct.
		aabb(vec_type minp, vec_type maxp) : min(minp), max(maxp) {
		}

		/// Returns the \ref aabb that contains all given points.
		template <typename ...Args> [[nodiscard]] inline static aabb containing(
			const vec_type &first, const Args &...args
		) {
			aabb result(first, first);
			(result.make_contain(args), ...);
			return result;
		}
		/// Returns the \ref aabb that contains all given points.
		template <typename It> [[nodiscard]] inline static aabb containing_dynamic(It &&begin, It &&end) {
			if (begin == end) {
				return aabb();
			}
			aabb result;
			result.min = result.max = *begin;
			std::decay_t<It> it = begin;
			for (++it; it != end; ++it) {
				result.make_contain(*it);
			}
			return result;
		}

		/// Adjusts \ref min and \ref max to make this \ref aabb contain the given point.
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
		[[nodiscard]] inline static bool intersects(const aabb &lhs, const aabb &rhs) {
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
		[[nodiscard]] inline static aabb intersection(const aabb &lhs, const aabb &rhs) {
			aabb result;
			result.min = vec_ops::apply<vec_type>(
				static_cast<const T & (*)(const T&, const T&)>(std::max), lhs.min, rhs.min
			);
			result.max = vec_ops::apply<vec_type>(
				static_cast<const T & (*)(const T&, const T&)>(std::min), lhs.max, rhs.max
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

	template <typename T> using aabb2 = aabb<2, T>; ///< 2D axis-aligned bounding boxes.
	using aabb2d = aabb2<double>; ///< 2D double axis-aligned bounding boxes.

	template <typename T> using aabb3 = aabb<3, T>; ///< 3D axis-aligned bounding boxes.
	using aabb3d = aabb3<double>; ///< 3D double axis-aligned bounding boxes.
}
