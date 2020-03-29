#include "fluid/voxelizer.h"

/// \file
/// Implementation of the voxelizer.

#include <algorithm>
#include <stack>

namespace fluid {
	// https://fileadmin.cs.lth.se/cs/Personal/Tomas_Akenine-Moller/pubs/tribox.pdf
	bool voxelizer::box_triangle_overlap(vec3d box_center, vec3d half_extent, vec3d p1, vec3d p2, vec3d p3) {
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

		return _box_triangle_overlap_no_aabb_center(half_extent, p1, p2, p3);
	}

	bool voxelizer::box_triangle_overlap_no_aabb(vec3d box_center, vec3d half_extent, vec3d p1, vec3d p2, vec3d p3) {
		p1 -= box_center;
		p2 -= box_center;
		p3 -= box_center;
		return _box_triangle_overlap_no_aabb_center(half_extent, p1, p2, p3);
	}

	bool voxelizer::_box_triangle_overlap_no_aabb_center(vec3d half_extent, vec3d p1, vec3d p2, vec3d p3) {
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

	void voxelizer::resize_reposition_grid(vec3d min, vec3d max) {
		vec3d
			size = max - min,
			grid_size = vec_ops::apply<vec3d>(static_cast<double (*)(double)>(std::ceil), size / cell_size);
		grid_offset = min - 0.5 * (grid_size * cell_size - size) - vec3d(cell_size, cell_size, cell_size);
		voxels = grid3<cell_type>(vec3s(grid_size) + vec3s(2, 2, 2), cell_type::interior);
	}

	vec3i voxelizer::resize_reposition_grid_constrained(
		vec3d min, vec3d max, double ref_cell_size, vec3d ref_grid_offset, vec3s grid_size
	) {
		cell_size = ref_cell_size;
		vec3i
			grid_min(vec_ops::apply<vec3d>(
				static_cast<double (*)(double)>(std::floor), (min - ref_grid_offset) / cell_size)
			),
			grid_max(vec_ops::apply<vec3d>(
				static_cast<double (*)(double)>(std::ceil), (max - ref_grid_offset) / cell_size)
			);
		grid_min -= vec3i(1, 1, 1);
		grid_max += vec3i(1, 1, 1);
		grid_offset = ref_grid_offset + vec3d(grid_min) * cell_size;
		voxels = grid3<cell_type>(vec3s(grid_max - grid_min), cell_type::interior);
		return grid_min;
	}

	void voxelizer::voxelize_triangle(vec3d p1, vec3d p2, vec3d p3) {
		vec3d min, max;
		_update_bounding_box(p1, min, max);
		_update_bounding_box(p2, min, max);
		_update_bounding_box(p3, min, max);

		vec3d half_extents = vec3d(cell_size, cell_size, cell_size) * 0.5;

		// here we assume the indices are in range
		// otherwise converting a negative float to unsigned is undefined behavior
		vec3s min_id((min - grid_offset) / cell_size), max_id((max - grid_offset) / cell_size);
		vec3d min_center = grid_offset + vec3d(min_id) * cell_size + half_extents, center = min_center;
		for (std::size_t z = min_id.z; z <= max_id.z; ++z, center.z += cell_size) {
			center.y = min_center.y;
			for (std::size_t y = min_id.y; y <= max_id.y; ++y, center.y += cell_size) {
				center.x = min_center.x;
				for (std::size_t x = min_id.x; x <= max_id.x; ++x, center.x += cell_size) {
					cell_type &type = voxels(x, y, z);
					if (type != cell_type::surface) {
						if (box_triangle_overlap_no_aabb(center, half_extents, p1, p2, p3)) {
							type = cell_type::surface;
						}
					}
				}
			}
		}
	}

	void voxelizer::mark_exterior() {
		if (voxels.get_array_size(voxels.get_size()) == 0) {
			return;
		}

		if (voxels(0, 0, 0) == cell_type::surface) {
			return;
		}
		voxels(0, 0, 0) = cell_type::exterior;
		std::stack<vec3s> stack;
		stack.emplace(vec3s(0, 0, 0));

		auto check_push = [this, &stack](vec3s p) {
			cell_type &ct = voxels(p);
			if (ct == cell_type::interior) {
				ct = cell_type::exterior;
				stack.emplace(p);
			}
		};

		while (stack.empty()) {
			vec3s cur = stack.top();
			stack.pop();

			if (cur.z > 0) {
				check_push(vec3s(cur.x, cur.y, cur.z - 1));
			}
			if (cur.y > 0) {
				check_push(vec3s(cur.x, cur.y - 1, cur.z));
			}
			if (cur.x > 0) {
				check_push(vec3s(cur.x - 1, cur.y, cur.z));
			}

			if (cur.z + 1 < voxels.get_size().z) {
				check_push(vec3s(cur.x, cur.y, cur.z + 1));
			}
			if (cur.y + 1 < voxels.get_size().y) {
				check_push(vec3s(cur.x, cur.y + 1, cur.z));
			}
			if (cur.x + 1 < voxels.get_size().x) {
				check_push(vec3s(cur.x + 1, cur.y, cur.z));
			}
		}
	}

	void voxelizer::_update_bounding_box(vec3d pos, vec3d &min, vec3d &max) {
		vec_ops::for_each(
			[](double v, double &minv, double &maxv) {
				minv = std::min(minv, v);
				maxv = std::max(maxv, v);
			},
			pos, min, max
				);
	}
}
