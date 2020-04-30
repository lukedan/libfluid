#include "grid_locator_node.h"

/// \file
/// Implementation of the grid manipulator node.

#include <maya/MFnPointArrayData.h>
#include <maya/MHWGeometryUtilities.h>
#include <maya/MHardwareRenderer.h>
#include <maya/MGLFunctionTable.h>
#include <maya/MFnPointArrayData.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>

#include "../misc.h"
#include "grid_node.h"

namespace fluid::maya {
	MObject
		grid_locator_node::attr_cell_size,
		grid_locator_node::attr_grid_size,
		grid_locator_node::attr_grid_offset,
		grid_locator_node::attr_particle_positions;

	void *grid_locator_node::creator() {
		return new grid_locator_node();
	}

	MStatus grid_locator_node::initialize() {
		MStatus stat;

		MFnNumericAttribute cell_size;
		attr_cell_size = cell_size.create("cellSize", "cell", MFnNumericData::kDouble, 1.0, &stat);
		FLUID_MAYA_CHECK(stat, "parameter creation");

		MFnNumericAttribute grid_size;
		attr_grid_size = grid_size.create("gridSize", "grid", MFnNumericData::k3Int, 0.0, &stat);
		FLUID_MAYA_CHECK(stat, "parameter creation");

		MFnNumericAttribute grid_offset;
		attr_grid_offset = grid_offset.create("gridOffset", "goff", MFnNumericData::k3Double, 0.0, &stat);
		FLUID_MAYA_CHECK(stat, "parameter creation");

		MFnTypedAttribute particle_positions;
		attr_particle_positions = particle_positions.create(
			"particlePositions", "ps", MFnData::kPointArray, MObject::kNullObj, &stat
		);
		FLUID_MAYA_CHECK(stat, "parameter creation");

		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_cell_size), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_grid_size), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_grid_offset), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_particle_positions), "parameter registration");

		return MStatus::kSuccess;
	}

	void grid_locator_node::draw(M3dView &view, const MDagPath&, M3dView::DisplayStyle, M3dView::DisplayStatus) {
		MStatus stat;

		auto [grid_offset, cell_size, grid_size] = _get_grid_attributes();

		MPlug particles_plug(thisMObject(), attr_particle_positions);
		MObject particles_obj;
		maya_check(particles_plug.getValue(particles_obj), FLUID_DEBUG_MESSAGE("prepare for draw"));
		MFnPointArrayData particles_data(particles_obj, &stat);
		maya_check(stat, FLUID_DEBUG_MESSAGE("prepare for draw"));
		unsigned int num_particles = particles_data.length(&stat);
		maya_check(stat, FLUID_DEBUG_MESSAGE("prepare for draw"));
		std::vector<vec3d> particles(static_cast<std::size_t>(num_particles));
		for (unsigned int i = 0; i < num_particles; ++i) {
			const MPoint &pt = particles_data[i];
			vec3d &target = particles[static_cast<std::size_t>(i)];
			target.x = pt.x;
			target.y = pt.y;
			target.z = pt.z;
		}

		maya_check(view.beginGL(), FLUID_DEBUG_MESSAGE("begin gl"));

		vec3d min = grid_offset, max = grid_offset + cell_size * vec3d(grid_size);
		MHardwareRenderer *renderer = MHardwareRenderer::theRenderer();
		MGLFunctionTable *gl = renderer->glFunctionTable();

		gl->glBegin(GL_LINES);
		gl->glVertex3d(min.x, min.y, min.z);
		gl->glVertex3d(max.x, min.y, min.z);
		gl->glVertex3d(max.x, min.y, min.z);
		gl->glVertex3d(max.x, max.y, min.z);
		gl->glVertex3d(max.x, max.y, min.z);
		gl->glVertex3d(min.x, max.y, min.z);
		gl->glVertex3d(min.x, max.y, min.z);
		gl->glVertex3d(min.x, min.y, min.z);

		gl->glVertex3d(min.x, min.y, min.z);
		gl->glVertex3d(min.x, min.y, max.z);
		gl->glVertex3d(max.x, min.y, min.z);
		gl->glVertex3d(max.x, min.y, max.z);
		gl->glVertex3d(max.x, max.y, min.z);
		gl->glVertex3d(max.x, max.y, max.z);
		gl->glVertex3d(min.x, max.y, min.z);
		gl->glVertex3d(min.x, max.y, max.z);

		gl->glVertex3d(min.x, min.y, max.z);
		gl->glVertex3d(max.x, min.y, max.z);
		gl->glVertex3d(max.x, min.y, max.z);
		gl->glVertex3d(max.x, max.y, max.z);
		gl->glVertex3d(max.x, max.y, max.z);
		gl->glVertex3d(min.x, max.y, max.z);
		gl->glVertex3d(min.x, max.y, max.z);
		gl->glVertex3d(min.x, min.y, max.z);
		gl->glEnd();

		gl->glBegin(GL_POINTS);
		for (vec3d pt : particles) {
			gl->glVertex3d(pt.x, pt.y, pt.z);
		}
		gl->glEnd();

		maya_check(view.endGL(), FLUID_DEBUG_MESSAGE("end gl"));
	}

	std::tuple<vec3d, double, vec3i> grid_locator_node::_get_grid_attributes() const {
		MStatus stat;

		vec3d grid_offset;
		double cell_size = 0.0;
		vec3i grid_size;

		MPlug grid_offset_plug(thisMObject(), attr_grid_offset);
		MObject grid_offset_obj;
		maya_check(grid_offset_plug.getValue(grid_offset_obj), FLUID_DEBUG_MESSAGE("bounding box calculation"));
		MFnNumericData grid_offset_data(grid_offset_obj, &stat);
		maya_check(stat, FLUID_DEBUG_MESSAGE("bounding box calculation"));
		maya_check(
			grid_offset_data.getData3Double(grid_offset.x, grid_offset.y, grid_offset.z),
			FLUID_DEBUG_MESSAGE("bounding box calculation")
		);

		MPlug cell_size_plug(thisMObject(), attr_cell_size);
		maya_check(cell_size_plug.getValue(cell_size), FLUID_DEBUG_MESSAGE("bounding box calculation"));

		MPlug grid_size_plug(thisMObject(), attr_grid_size);
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
