#pragma once

/// \file
/// Nodes used for the visualization of particles.

#include <maya/MPxLocatorNode.h>

#include <fluid/math/vec.h>

namespace fluid::maya {
	/// Draw override for \ref grid_node.
	class grid_locator_node : public MPxLocatorNode {
	public:
		static MObject
			attr_cell_size, ///< The cell size attribute.
			attr_grid_size, ///< The grid size attribute.
			attr_grid_offset, ///< The grid offset attribute.
			attr_particle_positions; ///< The particle positions attribute.
		const static MTypeId id; ///< The ID of this node.

		/// Draws the grid and particles.
		void draw(M3dView&, const MDagPath&, M3dView::DisplayStyle, M3dView::DisplayStatus) override;

		/// Creation function passed to \p MFnPlugin::registerNode().
		static void *creator();
		/// Initialization function passed to \p MFnPlugin::registerNode().
		static MStatus initialize();
	private:
		std::tuple<vec3d, double, vec3i> grid_locator_node::_get_grid_attributes() const;
	};
}
