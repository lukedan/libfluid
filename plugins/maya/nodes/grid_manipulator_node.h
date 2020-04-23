#pragma once

/// \file
/// Definition of the manipulator node for the simulation grid.

#include <maya/MPxManipContainer.h>

namespace fluid::maya {
	/// A manipulator node that displays the selectable boundaries of the grid.
	class grid_manipulator_node : public MPxManipContainer {
	public:
		const static MString
			attr_grid_offset_name, ///< The name of the grid offset value.
			attr_cell_size_name, ///< The name of the cell size value.
			attr_grid_size_name; ///< The name of the grid size value.
		const static MTypeId id; ///< The ID of this node.

		/// Called after the constructor returns to initialize \ref _object_name.
		void postConstructor() override;

		/// Called before the UI is drawn.
		void preDrawUI(const M3dView&) override;
		/// Draws this manipulator.
		void drawUI(MUIDrawManager&, const MFrameContext&) const override;

		/// Creation function passed to \p MFnPlugin::registerNode().
		static void *creator();
		/// Initialization function passed to \p MFnPlugin::registerNode().
		static MStatus initialize();
	private:
		MGLuint _object_name = 0; ///< Used for drawing and picking.
		int
			_grid_offset_value_index = -1, ///< Index of the grid offset value.
			_cell_size_value_index = -1, ///< Index of the cell size value.
			_grid_size_value_index = -1; ///< Index of the grid size value.

		MPoint _grid_offset; ///< Cached grid offset.
		double _cell_size = 0.0; ///< Cached cell size.
		MVector _grid_size; ///< Cached grid size.
	};
}
