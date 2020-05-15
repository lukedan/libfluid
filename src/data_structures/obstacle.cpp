#include "fluid/data_structures/obstacle.h"

/// \file
/// Implementation of obstacles.

#include "fluid/voxelizer.h"

namespace fluid {
	obstacle::obstacle(mesh_t m, double cell_size, vec3d ref_grid_offset, vec3s ref_grid_size) :
		obstacle_mesh(std::move(m)) {

		auto [rmin, rmax] = voxelizer::get_bounding_box(
			obstacle_mesh.positions.begin(), obstacle_mesh.positions.end()
		);
		voxelizer vox;
		vec3i offset = vox.resize_reposition_grid_constrained(rmin, rmax, cell_size, ref_grid_offset);
		vox.voxelize_mesh_surface(obstacle_mesh);
		vox.mark_exterior();

		auto [min_coord, max_coord] = vox.get_overlapping_cell_range(offset, ref_grid_size);
		vox.voxels.for_each_in_range_unchecked(
			[this, offset](vec3s pos, voxelizer::cell_type type) {
				if (type == voxelizer::cell_type::interior) {
					cells.emplace_back(vec3s(vec3i(pos) + offset));
				}
			},
			min_coord, max_coord
				);
	}
}
