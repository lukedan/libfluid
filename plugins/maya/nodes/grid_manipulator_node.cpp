#include "grid_manipulator_node.h"

/// \file
/// Implementation of the grid manipulator node.

#include <maya/MFnPointArrayData.h>
#include <maya/MHWGeometryUtilities.h>

#include "../misc.h"
#include "grid_node.h"

namespace fluid::maya {
	void *grid_manipulator_node::creator() {
		return new grid_manipulator_node();
	}

	MStatus grid_manipulator_node::initialize() {
		return MStatus::kSuccess;
	}

	MStatus grid_manipulator_node::connectToDependNode(const MObject &node) {
		_grid_node = node;
		FLUID_MAYA_CHECK_RETURN(finishAddingManips(), "connect to grid node");
		return MPxManipContainer::connectToDependNode(node);
	}

	void grid_manipulator_node::preDrawUI(const M3dView&) {
		MStatus stat;

		auto [grid_offset, cell_size, grid_size] = _get_grid_attributes();
		_grid_min = grid_offset;
		_grid_max = grid_offset + cell_size * vec3d(grid_size);

		MPlug particles_plug(_grid_node, grid_node::attr_output_particle_positions);
		MObject particles_obj;
		maya_check(particles_plug.getValue(particles_obj), FLUID_DEBUG_MESSAGE("prepare for draw"));
		MFnPointArrayData particles_data(particles_obj, &stat);
		maya_check(stat, FLUID_DEBUG_MESSAGE("prepare for draw"));
		unsigned int num_particles = particles_data.length(&stat);
		maya_check(stat, FLUID_DEBUG_MESSAGE("prepare for draw"));
		_particles.resize(static_cast<std::size_t>(num_particles));
		for (unsigned int i = 0; i < num_particles; ++i) {
			const MPoint &pt = particles_data[i];
			vec3d &target = _particles[static_cast<std::size_t>(i)];
			target.x = pt.x;
			target.y = pt.y;
			target.z = pt.z;
		}
	}

	void grid_manipulator_node::drawUI(MUIDrawManager &ui, const MFrameContext&) const {
		vec3d min = _grid_min, max = _grid_max;
		ui.beginDrawable();
		/*ui.setColor();*/

		ui.line(MPoint(min.x, min.y, min.z), MPoint(max.x, min.y, min.z));
		ui.line(MPoint(max.x, min.y, min.z), MPoint(max.x, max.y, min.z));
		ui.line(MPoint(max.x, max.y, min.z), MPoint(min.x, max.y, min.z));
		ui.line(MPoint(min.x, max.y, min.z), MPoint(min.x, min.y, min.z));

		ui.line(MPoint(min.x, min.y, min.z), MPoint(min.x, min.y, max.z));
		ui.line(MPoint(max.x, min.y, min.z), MPoint(max.x, min.y, max.z));
		ui.line(MPoint(max.x, max.y, min.z), MPoint(max.x, max.y, max.z));
		ui.line(MPoint(min.x, max.y, min.z), MPoint(min.x, max.y, max.z));

		ui.line(MPoint(min.x, min.y, max.z), MPoint(max.x, min.y, max.z));
		ui.line(MPoint(max.x, min.y, max.z), MPoint(max.x, max.y, max.z));
		ui.line(MPoint(max.x, max.y, max.z), MPoint(min.x, max.y, max.z));
		ui.line(MPoint(min.x, max.y, max.z), MPoint(min.x, min.y, max.z));

		ui.endDrawable();

		ui.beginDrawable();
		for (vec3d pt : _particles) {
			ui.point(MPoint(pt.x, pt.y, pt.z));
		}
		ui.endDrawable();
	}

	std::tuple<vec3d, double, vec3i> grid_manipulator_node::_get_grid_attributes() const {
		MStatus stat;

		vec3d grid_offset;
		double cell_size = 0.0;
		vec3i grid_size;

		MPlug grid_offset_plug(_grid_node, grid_node::attr_grid_offset);
		MObject grid_offset_obj;
		maya_check(grid_offset_plug.getValue(grid_offset_obj), FLUID_DEBUG_MESSAGE("bounding box calculation"));
		MFnNumericData grid_offset_data(grid_offset_obj, &stat);
		maya_check(stat, FLUID_DEBUG_MESSAGE("bounding box calculation"));
		maya_check(
			grid_offset_data.getData3Double(grid_offset.x, grid_offset.y, grid_offset.z),
			FLUID_DEBUG_MESSAGE("bounding box calculation")
		);

		MPlug cell_size_plug(_grid_node, grid_node::attr_cell_size);
		maya_check(cell_size_plug.getValue(cell_size), FLUID_DEBUG_MESSAGE("bounding box calculation"));

		MPlug grid_size_plug(_grid_node, grid_node::attr_grid_size);
		MObject grid_size_obj;
		maya_check(grid_size_plug.getValue(grid_size_obj), FLUID_DEBUG_MESSAGE("bounding box calculation"));
		MFnNumericData grid_size_data(grid_size_obj, &stat);
		maya_check(stat, FLUID_DEBUG_MESSAGE("bounding box calculation"));
		maya_check(
			grid_size_data.getData3Int(grid_size.x, grid_size.y, grid_size.z),
			FLUID_DEBUG_MESSAGE("bounding box calculation")
		);

		return { grid_offset, cell_size, grid_size };
	}
}
