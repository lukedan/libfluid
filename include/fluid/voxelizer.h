#pragma once

/// \file
/// Declaration of the voxelizer.

#include <algorithm>

#include "math/vec.h"
#include "data_structures/grid.h"
#include "data_structures/mesh.h"

namespace fluid {
	/// Used to voxelize meshes.
	class voxelizer {
	public:
		/// The type of a cell.
		enum class cell_type : unsigned char {
			interior, ///< The cell lies completely inside the mesh.
			exterior, ///< The cell lies outside of the mesh.
			surface ///< The cell intersects with the surface of the mesh.
		};

		/// Tests if the given axis-aligned box overlaps the given triangle.
		[[nodiscard]] static bool box_triangle_overlap(
			vec3d box_center, vec3d half_extent, vec3d p1, vec3d p2, vec3d p3
		);
		/// Tests if the given axis-aligned box overlaps the given triangle, assuming that the box overlaps with the
		/// triangle's AABB.
		[[nodiscard]] static bool box_triangle_overlap_no_aabb(
			vec3d box_center, vec3d half_extent, vec3d p1, vec3d p2, vec3d p3
		);
		/// Returns the bounding box of the given list of vertices.
		template <typename It> inline static std::pair<vec3d, vec3d> get_bounding_box(It beg, It end) {
			if (beg == end) {
				return { vec3d(), vec3d() };
			}
			vec3d min(*beg), max(*beg);
			auto it = beg;
			for (++it; it != end; ++it) {
				_update_bounding_box(vec3d(*it), min, max);
			}
			return { min, max };
		}

		/// Sets \ref grid_offset and the size of \ref voxels based on the given bounding box. This function clears
		/// the contents of \ref voxels.
		void resize_reposition_grid(vec3d min, vec3d max);
		/// Sets \ref grid_offset, \ref cell_size, and the size of \ref voxels based on the given bounding box,
		/// making sure that the grid aligns with the specified grid. This function clears the contents of
		/// \ref voxels.
		///
		/// \return The offset of \ref voxels in the given grid.
		vec3i resize_reposition_grid_constrained(
			vec3d min, vec3d max, double ref_cell_size, vec3d ref_grid_offset, vec3s grid_size
		);

		/// Returns the range of cells that overlap a reference grid. The input is the result of
		/// \ref resize_reposition_grid_constrained() and the size of the reference grid.
		std::pair<vec3s, vec3s> get_overlapping_cell_range(vec3i, vec3s) const;

		/// Marks cells that overlap the given triangle as surface cells. The triangle must be entirely contained by
		/// \ref voxels.
		void voxelize_triangle(vec3d, vec3d, vec3d);
		/// Marks surface cells in \ref voxels. The mesh must be entirely contained by \ref voxels, which can be made
		/// sure by calling \ref resize_reposition_grid() first.
		template <typename Mesh> void voxelize_mesh_surface(const Mesh &mesh) {
			for (std::size_t i = 0; i + 2 < mesh.indices.size(); i += 3) {
				voxelize_triangle(
					vec3d(mesh.positions[mesh.indices[i]]),
					vec3d(mesh.positions[mesh.indices[i + 1]]),
					vec3d(mesh.positions[mesh.indices[i + 2]])
				);
			}
		}

		/// Marks the exterior cells.
		void mark_exterior();

		double cell_size = 1.0; ///< The size of each cell.
		grid3<cell_type> voxels; ///< The voxels.
		vec3d grid_offset; ///< The offset of \ref voxels.
	private:
		/// Tests if the given axis-aligned box overlaps the given triangle, assuming that the box's center is at
		/// the origin and that the box overlaps with the triangle's AABB.
		static bool _box_triangle_overlap_no_aabb_center(vec3d half_extent, vec3d p1, vec3d p2, vec3d p3);

		/// Updates the bounding box.
		static void _update_bounding_box(vec3d pos, vec3d &min, vec3d &max);
	};
}
