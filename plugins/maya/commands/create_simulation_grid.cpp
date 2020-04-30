#include "create_simulation_grid.h"

/// \file
/// Implementation of the \p createSimulationGrid command.

#include <maya/MStatus.h>
#include <maya/MArgList.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MItDependencyNodes.h>
#include <maya/MFnMesh.h>
#include <maya/MDagPath.h>

#include "../misc.h"
#include "../nodes/grid_node.h"
#include "../nodes/grid_locator_node.h"
#include "../nodes/mesher_node.h"

namespace fluid::maya {
	void *create_simulation_grid_command::creator() {
		return new create_simulation_grid_command();
	}

	MStatus create_simulation_grid_command::doIt(const MArgList &args) {
		MStatus stat;

		MObject grid = _graph_modifier.MDGModifier::createNode(grid_node::id, &stat);
		FLUID_MAYA_CHECK(stat, "node creation");

		MObject mesher = _graph_modifier.MDGModifier::createNode(mesher_node::id, &stat);
		FLUID_MAYA_CHECK(stat, "node creation");

		/*FLUID_MAYA_CHECK_RETURN(_graph_modifier_dg.doIt(), "execute queued operations");*/

		MFnDependencyNode fn_mesher(mesher, &stat);
		FLUID_MAYA_CHECK(stat, "plug retrieval");

		MObject locator = _graph_modifier.createNode(grid_locator_node::id, MObject::kNullObj, &stat);
		FLUID_MAYA_CHECK(stat, "node creation");
		MFnDagNode fn_locator(locator, &stat);
		FLUID_MAYA_CHECK(stat, "plug retrieval");

		// time -> grid
		MItDependencyNodes time_iter(MFn::kTime, &stat);
		FLUID_MAYA_CHECK(stat, "plug retrieval");
		bool has_time = !time_iter.isDone(&stat);
		FLUID_MAYA_CHECK(stat, "plug retrieval");
		if (has_time) {
			MObject time = time_iter.thisNode(&stat);
			FLUID_MAYA_CHECK(stat, "plug retrieval");
			MFnDependencyNode fn_time(time, &stat);
			FLUID_MAYA_CHECK(stat, "plug retrieval");
			MPlug time_plug = fn_time.findPlug("outTime", false, &stat);
			FLUID_MAYA_CHECK(stat, "plug retrieval");

			MFnDependencyNode fn_grid(grid, &stat);
			FLUID_MAYA_CHECK(stat, "plug retrieval");
			MPlug grid_time_plug = fn_grid.findPlug(grid_node::attr_time, false, &stat);
			FLUID_MAYA_CHECK(stat, "plug retrieval");

			FLUID_MAYA_CHECK_RETURN(_graph_modifier.connect(time_plug, grid_time_plug), "attribute connection");
		} else {
			// TODO print warning
		}

		// grid -> mesher
		FLUID_MAYA_CHECK_RETURN(
			_graph_modifier.connect(
				grid, grid_node::attr_output_particle_positions,
				mesher, mesher_node::attr_particles
			),
			"attribute connection"
		);

		// grid -> locator
		FLUID_MAYA_CHECK_RETURN(
			_graph_modifier.connect(
				grid, grid_node::attr_cell_size,
				locator, grid_locator_node::attr_cell_size
			),
			"attribute connection"
		);
		FLUID_MAYA_CHECK_RETURN(
			_graph_modifier.connect(
				grid, grid_node::attr_grid_offset,
				locator, grid_locator_node::attr_grid_offset
			),
			"attribute connection"
		);
		FLUID_MAYA_CHECK_RETURN(
			_graph_modifier.connect(
				grid, grid_node::attr_grid_size,
				locator, grid_locator_node::attr_grid_size
			),
			"attribute connection"
		);
		FLUID_MAYA_CHECK_RETURN(
			_graph_modifier.connect(
				grid, grid_node::attr_output_particle_positions,
				locator, grid_locator_node::attr_particle_positions
			),
			"attribute connection"
		);

		// mesher -> mesh
		MPlug mesher_output = fn_mesher.findPlug(mesher_node::attr_output_mesh, false, &stat);
		FLUID_MAYA_CHECK(stat, "plug retrieval");

		MFnMesh mesh_creator;
		MPointArray points;
		MIntArray counts, connects;
		MObject transform = mesh_creator.create(0, 0, points, counts, connects, MObject::kNullObj, &stat);
		FLUID_MAYA_CHECK(stat, "mesh creation");
		MFnDagNode fn_transform(transform, &stat);
		FLUID_MAYA_CHECK(stat, "mesh creation");
		MObject shape = fn_transform.child(0, &stat);
		FLUID_MAYA_CHECK(stat, "mesh creation");
		MFnDagNode fn_shape(shape, &stat);
		FLUID_MAYA_CHECK(stat, "mesh creation");
		MPlug mesh_input = fn_shape.findPlug("inMesh", false, &stat);
		FLUID_MAYA_CHECK(stat, "plug retrieval");
		FLUID_MAYA_CHECK_RETURN(_graph_modifier.connect(mesher_output, mesh_input), "attribute connection");

		FLUID_MAYA_CHECK_RETURN(_graph_modifier.doIt(), "execute queued operations");

		return MStatus::kSuccess;
	}

	MStatus create_simulation_grid_command::undoIt() {
		FLUID_MAYA_CHECK_RETURN(_graph_modifier.undoIt(), "undo");
		return MStatus::kSuccess;
	}

	MStatus create_simulation_grid_command::redoIt() {
		FLUID_MAYA_CHECK_RETURN(_graph_modifier.doIt(), "redo");
		return MStatus::kSuccess;
	}
}
