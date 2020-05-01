#pragma once

/// \file
/// Nodes used for the visualization of particles.

#include <maya/MPxManipContainer.h>

#include <fluid/math/vec.h>

namespace fluid::maya {
	/// Draw override for \ref grid_node.
	class grid_manipulator_node : public MPxManipContainer {
	public:
		static MObject
			attr_cell_size, ///< The cell size attribute.
			attr_grid_size, ///< The grid size attribute.
			attr_grid_offset, ///< The grid offset attribute.
			attr_particle_positions; ///< The particle positions attribute.
		const static MTypeId id; ///< The ID of this node.

		/// Connects this manipulator to the given dependency node.
		MStatus connectToDependNode(const MObject&) override;

		/// Saves data for \ref drawUI().
		void preDrawUI(const M3dView&) override;
		/// Renders the grid.
		void drawUI(MUIDrawManager&, const MFrameContext&) const override;

		/// Creation function passed to \p MFnPlugin::registerNode().
		static void *creator();
		/// Initialization function passed to \p MFnPlugin::registerNode().
		static MStatus initialize();
	private:
		MObject _grid_node; ///< The grid node.

		std::vector<vec3d> _particles; ///< All particles.
		vec3d
			_grid_min, ///< The minimum corner of the grid.
			_grid_max; ///< The maximum corner of the grid.

		/// Retrieves and returns all grid attributes.
		std::tuple<vec3d, double, vec3i> _get_grid_attributes() const;
	};
}
