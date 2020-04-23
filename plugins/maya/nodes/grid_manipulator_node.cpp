#include "grid_manipulator_node.h"

/// \file
/// Implementation of the grid manipulator node.

#include "../misc.h"

namespace fluid::maya {
	const MString
		grid_manipulator_node::attr_grid_offset_name{ "gridOffset" },
		grid_manipulator_node::attr_cell_size_name{ "cellSize" },
		grid_manipulator_node::attr_grid_size_name{ "gridSize" };

	void *grid_manipulator_node::creator() {
		return new grid_manipulator_node();
	}

	MStatus grid_manipulator_node::initialize() {
		return MStatus::kSuccess;
	}

	void grid_manipulator_node::postConstructor() {
		maya_check(glFirstHandle(_object_name), FLUID_DEBUG_MESSAGE("get object handle"));

		maya_check(
			addPointValue(attr_grid_offset_name, MPoint(0.0, 0.0, 0.0), _grid_offset_value_index),
			FLUID_DEBUG_MESSAGE("add value")
		);
		maya_check(
			addDoubleValue(attr_cell_size_name, 0.5, _cell_size_value_index),
			FLUID_DEBUG_MESSAGE("add value")
		);
		maya_check(
			addVectorValue(attr_grid_size_name, MVector(50.0, 50.0, 50.0), _grid_size_value_index),
			FLUID_DEBUG_MESSAGE("add value")
		);
	}

	void grid_manipulator_node::preDrawUI(const M3dView&) {
		maya_check(getPointValue(_grid_offset_value_index, false, _grid_offset), FLUID_DEBUG_MESSAGE("get value"));
		maya_check(getDoubleValue(_cell_size_value_index, false, _cell_size), FLUID_DEBUG_MESSAGE("get value"));
		maya_check(getVectorValue(_grid_size_value_index, false, _grid_size), FLUID_DEBUG_MESSAGE("get value"));
	}

	void grid_manipulator_node::drawUI(
		MUIDrawManager &draw_manager, const MFrameContext &frame_context
	) const {
		MVector
			xoff(_cell_size * _grid_size.x, 0.0, 0.0),
			yoff(0.0, _cell_size * _grid_size.y, 0.0),
			zoff(0.0, 0.0, _cell_size * _grid_size.z);
		MPoint
			x = _grid_offset + xoff, y = _grid_offset + yoff, z = _grid_offset + zoff,
			xy = _grid_offset + xoff + yoff, xz = _grid_offset + xoff + zoff, yz = _grid_offset + yoff + zoff,
			xyz = _grid_offset + xoff + yoff + zoff;

		draw_manager.beginDrawable(MUIDrawManager::kSelectable, _object_name);
		maya_check(
			setHandleColor(draw_manager, _object_name, lineColor()),
			FLUID_DEBUG_MESSAGE("draw")
		);
		draw_manager.line(_grid_offset, x);
		draw_manager.line(_grid_offset, y);
		draw_manager.line(_grid_offset, z);
		draw_manager.line(x, xy);
		draw_manager.line(x, xz);
		draw_manager.line(y, xy);
		draw_manager.line(y, yz);
		draw_manager.line(z, xz);
		draw_manager.line(z, yz);
		draw_manager.line(xy, xyz);
		draw_manager.line(xz, xyz);
		draw_manager.line(yz, xyz);
		draw_manager.endDrawable();
	}
}
