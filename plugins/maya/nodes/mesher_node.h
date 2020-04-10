#pragma once

/// \file
/// Declaration of the grid node.

#include <maya/MPxNode.h>

namespace fluid::maya {
	/// The mesher node.
	class mesher_node : public MPxNode {
	public:
		static MObject
			attr_cell_size, ///< The cell size node attribute.
			attr_grid_size, ///< The grid size node attribute.
			attr_grid_offset, ///< The particle offset attribute.
			attr_particles, ///< The particles attribute.
			attr_particle_size, ///< The particle size attribute.
			attr_particle_extents, ///< The particle extents attribute.
			attr_particle_check_radius, ///< The particle check radius attribute.
			attr_output_mesh; ///< The output mesh node attribute.
		static MTypeId id; ///< The ID of this node.

		/// Updates the mesh.
		MStatus compute(const MPlug&, MDataBlock&) override;

		/// Creation function passed to \p MFnPlugin::registerNode().
		static void *creator();
		/// Initialization function passed to \p MFnPlugin::registerNode().
		static MStatus initialize();
	};
}
