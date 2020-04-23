#pragma once

/// \file
/// Definition of source nodes.

#include <maya/MPxNode.h>

namespace fluid::maya {
	/// A fluid source.
	class voxelizer_node : public MPxNode {
	public:
		static MObject
			attr_input_mesh, ///< The input mesh.
			attr_cell_size, ///< The velocity of the particles.
			attr_ref_grid_offset, ///< The offset of the reference grid.
			attr_ref_grid_size, ///< The number of cells on each axis of the reference grid.
			attr_include_interior, ///< Indicates whether to include interior cells.
			attr_include_surface, ///< Indicates whether to include surface cells.

			attr_output_grid_offset, ///< The offset of the voxelization grid.
			attr_output_grid_size, ///< The size of the voxelization grid.
			/// Cells occupied by the mesh. This will be stored as a int array and the indices will be relative to
			/// the voxelization grid.
			attr_output_cells,
			/// Similar to \ref attr_output_cells, but with only cells in the reference grid. The indices will also
			/// be relative to the reference grid.
			attr_output_cells_ref;
		const static MTypeId id; ///< The ID of this node.

		/// Forwards the attributes to the output attribute.
		MStatus compute(const MPlug&, MDataBlock&) override;

		/// Creation function passed to \p MFnPlugin::registerNode().
		static void *creator();
		/// Initialization function passed to \p MFnPlugin::registerNode().
		static MStatus initialize();
	};
}
