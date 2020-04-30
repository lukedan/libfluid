#include "fluid/voxelizer.h"

/// \file
/// Implementation of the voxelizer.

#include <algorithm>
#include <stack>

#include "fluid/math/intersection.h"

namespace fluid {
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

	std::pair<vec3s, vec3s> voxelizer::get_overlapping_cell_range(vec3i offset, vec3s ref_grid_size) const {
		vec3s min_coord = vec_ops::apply<vec3s>(
			[](int coord) {
				return coord < 0 ? static_cast<std::size_t>(-coord) : 0;
			},
			offset
				);
		vec3s max_coord = vec_ops::apply<vec3s>(
			[](int coord, std::size_t max) {
				return std::min(static_cast<std::size_t>(std::max(0, coord)), max);
			},
			offset + vec3i(voxels.get_size()), ref_grid_size
				);
		return { min_coord, max_coord };
	}

	void voxelizer::voxelize_triangle(vec3d p1, vec3d p2, vec3d p3) {
		vec3d min = p1, max = p1;
		_update_bounding_box(p2, min, max);
		_update_bounding_box(p3, min, max);

		double half_cell_size = 0.5 * cell_size;
		vec3d half_extents = vec3d(half_cell_size, half_cell_size, half_cell_size);

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

		while (!stack.empty()) {
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
