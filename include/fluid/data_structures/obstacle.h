#pragma once

/// \file
/// Definition of static obstacles.

#include "mesh.h"

namespace fluid {
	/// A static obstacle.
	class obstacle {
	public:
		using mesh_t = mesh<double, std::size_t, double, double, vec3d>; ///< The mesh type.

		/// Default constructor.
		obstacle() = default;
		/// Constructs this obstacle by voxelizing the given mesh.
		obstacle(mesh_t, double cell_size, vec3d ref_grid_offset, vec3s ref_grid_size);

		mesh_t obstacle_mesh; ///< The mesh of this obstacle.
		std::vector<vec3s> cells; ///< Cells that are entirely occupied by this obstacle.
	};
}
