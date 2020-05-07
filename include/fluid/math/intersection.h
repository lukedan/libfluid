#pragma once

/// \file
/// Declaration of various intersection tests.

#include <optional>

#include "vec.h"

namespace fluid {
	/// Tests if the given axis-aligned box overlaps the given triangle.
	[[nodiscard]] bool aab_triangle_overlap(
		vec3d box_center, vec3d half_extent, vec3d p1, vec3d p2, vec3d p3
	);
	/// Tests if the given axis-aligned box overlaps the given triangle, assuming that the box overlaps with the
	/// triangle's AABB.
	[[nodiscard]] bool aab_triangle_overlap_bounded(
		vec3d box_center, vec3d half_extent, vec3d p1, vec3d p2, vec3d p3
	);
	/// Tests if the given axis-aligned box overlaps the given triangle, assuming that the box's center is at
	/// the origin and that the box overlaps with the triangle's AABB.
	[[nodiscard]] bool aab_triangle_overlap_bounded_center(vec3d half_extent, vec3d p1, vec3d p2, vec3d p3);

	/// Tests for intersection between a ray and a triangle.
	///
	/// \return If there's an intersection at \p p, the returned vector (x, y, z) will satisfy:
	///         <tt>p = origin + x * direction = p1 + y * (p2 - p1) + z * (p3 - p1)</tt>. Otherwise, the \p x
	///         component will be \p nan.
	[[nodiscard]] vec3d ray_triangle_intersection(
		vec3d origin, vec3d direction, vec3d p1, vec3d p2, vec3d p3, double parallel_epsilon = 1e-6
	);
	/// Tests for intersection between a ray and a triangle. The triangle is represented by the first vertex and the
	/// offsets of the other two vertices from the first one.
	[[nodiscard]] vec3d ray_triangle_intersection_edges(
		vec3d origin, vec3d direction, vec3d p1, vec3d e12, vec3d e13, double parallel_epsilon = 1e-6
	);

	/// Returns whether the ray overlaps the given axis-aligned box.
	///
	/// \return \p t values of the entry and exit points. If the ray does not intersect the box, then its \p x
	///         component will be \p nan. It is possible for the ray to start inside the box, in which case the \p x
	///         component will be less than zero.
	[[nodiscard]] vec2d aab_ray_intersection(vec3d min, vec3d max, vec3d vo, vec3d vd);

	/// Computes the intersection between the given ray and a sphere of radius 1 at the origin.
	///
	/// \return The return value is similar to that of \ref aab_ray_intersection().
	[[nodiscard]] vec2d unit_radius_sphere_ray_intersection(vec3d vo, vec3d vd);
}
