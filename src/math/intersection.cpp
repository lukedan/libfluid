#include "fluid/math/intersection.h"

/// \file
/// Implementation of various intersection algorithms.

#include <algorithm>

namespace fluid {
	// https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/pubs/tribox.pdf
	bool box_triangle_overlap(vec3d box_center, vec3d half_extent, vec3d p1, vec3d p2, vec3d p3) {
		p1 -= box_center;
		p2 -= box_center;
		p3 -= box_center;

		auto [xmin, xmax] = std::minmax({ p1.x, p2.x, p3.x });
		if (xmin > half_extent.x || xmax < half_extent.x) {
			return false;
		}
		auto [ymin, ymax] = std::minmax({ p1.y, p2.y, p3.y });
		if (ymin > half_extent.y || ymax < half_extent.y) {
			return false;
		}
		auto [zmin, zmax] = std::minmax({ p1.z, p2.z, p3.z });
		if (zmin > half_extent.z || zmax < half_extent.z) {
			return false;
		}

		return box_triangle_overlap_no_aabb_center(half_extent, p1, p2, p3);
	}

	bool box_triangle_overlap_no_aabb(vec3d box_center, vec3d half_extent, vec3d p1, vec3d p2, vec3d p3) {
		p1 -= box_center;
		p2 -= box_center;
		p3 -= box_center;
		return box_triangle_overlap_no_aabb_center(half_extent, p1, p2, p3);
	}

	bool box_triangle_overlap_no_aabb_center(vec3d half_extent, vec3d p1, vec3d p2, vec3d p3) {
		vec3d f[]{ p2 - p1, p3 - p2, p1 - p3 }, normal = vec_ops::cross(f[0], f[1]);
		double
			center_off = vec_ops::dot(p1, normal),
			radius_n = vec_ops::dot(
				vec_ops::apply<vec3d>(static_cast<double (*)(double)>(std::abs), normal), half_extent
			);
		if (std::abs(center_off) > std::abs(radius_n)) {
			return false;
		}

		vec3d v[]{ p1, p2, p3 };
		// (1, 0, 0) -> (0, -z, y)
		for (std::size_t i = 0; i < 3; ++i) {
			vec3d v1 = v[i], v2 = v[(i + 2) % 3], fi = f[i];
			double p0 = v1.z * fi.y - v1.y * fi.z, p1 = v2.z * fi.y - v2.y * fi.z;
			auto [pmin, pmax] = std::minmax(p0, p1);
			double r = half_extent.y * std::abs(fi.z) + half_extent.z * std::abs(fi.y);
			if (pmin > r || pmax < -r) {
				return false;
			}
		}
		// (0, 1, 0) -> (z, 0, -x)
		for (std::size_t i = 0; i < 3; ++i) {
			vec3d v1 = v[i], v2 = v[(i + 2) % 3], fi = f[i];
			double p0 = v1.x * fi.z - v1.z * fi.x, p1 = v2.x * fi.z - v2.z * fi.x;
			auto [pmin, pmax] = std::minmax(p0, p1);
			double r = half_extent.x * std::abs(fi.z) + half_extent.z * std::abs(fi.x);
			if (pmin > r || pmax < -r) {
				return false;
			}
		}
		// (0, 0, 1) -> (-y, x, 0)
		for (std::size_t i = 0; i < 3; ++i) {
			vec3d v1 = v[i], v2 = v[(i + 2) % 3], fi = f[i];
			double p0 = v1.y * fi.x - v1.x * fi.y, p1 = v2.y * fi.x - v2.x * fi.y;
			auto [pmin, pmax] = std::minmax(p0, p1);
			double r = half_extent.x * std::abs(fi.y) + half_extent.y * std::abs(fi.x);
			if (pmin > r || pmax < -r) {
				return false;
			}
		}

		return true;
	}


	// Moller-Trumbore
	std::optional<vec3d> ray_triangle_intersection(
		vec3d origin, vec3d direction, vec3d p1, vec3d p2, vec3d p3, double parallel_epsilon
	) {
		return ray_triangle_intersection_edges(origin, direction, p1, p2 - p1, p3 - p1, parallel_epsilon);
	}

	std::optional<vec3d> ray_triangle_intersection_edges(
		vec3d origin, vec3d direction, vec3d p1, vec3d e12, vec3d e13, double parallel_epsilon
	) {
		vec3d pvec = vec_ops::cross(direction, e13);
		double det = vec_ops::dot(e12, pvec);
		if (std::abs(det) < parallel_epsilon) { // return if parallel
			return std::nullopt;
		}
		double inv_det = 1.0 / det;

		vec3d e1o = origin - p1;
		double u = vec_ops::dot(e1o, pvec) * inv_det;
		if (u < 0.0 || u > 1.0) {
			return std::nullopt;
		}

		vec3d qvec = vec_ops::cross(e1o, e12);
		double v = vec_ops::dot(direction, qvec) * inv_det;
		if (v < 0.0 || u + v > 1.0) {
			return std::nullopt;
		}

		double t = vec_ops::dot(e13, qvec) * inv_det;
		if (t > 0.0) {
			return vec3d(t, u, v);
		}
		return std::nullopt;
	}
}
