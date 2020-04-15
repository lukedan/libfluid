#include "add_fluid_source.h"

/// \file
/// Implementation of the addFluidSource command.

#include <maya/MGlobal.h>
#include <maya/MSelectionList.h>
#include <maya/MDagPath.h>
#include <maya/MFnDependencyNode.h>

#include "../misc.h"
#include "../nodes/grid_node.h"
#include "../nodes/voxelizer_node.h"

namespace fluid::maya {
	void *add_fluid_source_command::creator() {
		return new add_fluid_source_command();
	}

	MStatus add_fluid_source_command::doIt(const MArgList &args) {
		MStatus stat;

		// get selected grid and geometry
		MSelectionList selection;
		FLUID_MAYA_CHECK_RETURN(MGlobal::getActiveSelectionList(selection), "retrieve selection");
		MObject grid = MObject::kNullObj;
		std::vector<MPlug> mesh_plugs;
		for (unsigned int i = 0; i < selection.length(); ++i) {
			{ // check for fluid grid
				MObject node;
				FLUID_MAYA_CHECK_RETURN(selection.getDependNode(i, node), "retrieve selection");
				MFnDependencyNode fn_node(node, &stat);
				FLUID_MAYA_CHECK(stat, "retrieve selection");
				MTypeId id = fn_node.typeId(&stat);
				FLUID_MAYA_CHECK(stat, "retrieve selection");
				if (id == grid_node::id) {
					if (grid.isNull()) {
						grid = std::move(node);
					} else {
						// TODO print error message
					}
					continue;
				}
			}
			{ // check for meshes
				MDagPath dag_path;
				FLUID_MAYA_CHECK_RETURN(selection.getDagPath(i, dag_path), "retrieve selection");
				FLUID_MAYA_CHECK_RETURN(dag_path.extendToShape(), "retrieve selection");
				MObject node = dag_path.node(&stat);
				FLUID_MAYA_CHECK(stat, "retrieve selection");
				MFnDependencyNode fn_node(node, &stat);
				FLUID_MAYA_CHECK(stat, "retrieve selection");
				MPlug mesh_plug = fn_node.findPlug("worldMesh", false, &stat);
				FLUID_MAYA_CHECK(stat, "retrieve selection");
				mesh_plugs.emplace_back(mesh_plug.elementByLogicalIndex(0, &stat));
				FLUID_MAYA_CHECK(stat, "retrieve selection");
				continue;
			}
			// TODO print error message
		}

		if (grid.isNull()) { // no grid selected
			// TODO print error message
			return MStatus::kInvalidParameter;
		}

		MFnDependencyNode fn_grid(grid, &stat);
		FLUID_MAYA_CHECK(stat, "plug retrieval");
		MPlug sources_plug = fn_grid.findPlug(grid_node::attr_sources, false, &stat);
		FLUID_MAYA_CHECK(stat, "plug retrieval");
		MPlug ref_cell_size_plug = fn_grid.findPlug(grid_node::attr_cell_size, false, &stat);
		FLUID_MAYA_CHECK(stat, "plug retrieval");
		MPlug ref_grid_offset_plug = fn_grid.findPlug(grid_node::attr_grid_offset, false, &stat);
		FLUID_MAYA_CHECK(stat, "plug retrieval");
		MPlug ref_grid_size_plug = fn_grid.findPlug(grid_node::attr_grid_size, false, &stat);
		FLUID_MAYA_CHECK(stat, "plug retrieval");

		unsigned int num_sources = sources_plug.numElements(&stat);
		FLUID_MAYA_CHECK(stat, "attribute connection");

		for (MPlug &mesh_plug : mesh_plugs) {
			MObject voxelizer = _graph_modifier.createNode(voxelizer_node::id, &stat);
			FLUID_MAYA_CHECK(stat, "node creation");

			MFnDependencyNode fn_voxelizer(voxelizer, &stat);
			FLUID_MAYA_CHECK(stat, "plug retrieval");
			MPlug vox_mesh_plug = fn_voxelizer.findPlug(voxelizer_node::attr_input_mesh, false, &stat);
			FLUID_MAYA_CHECK(stat, "plug retrieval");
			MPlug vox_cell_size_plug = fn_voxelizer.findPlug(voxelizer_node::attr_cell_size, false, &stat);
			FLUID_MAYA_CHECK(stat, "plug retrieval");
			MPlug vox_ref_grid_offset_plug =
				fn_voxelizer.findPlug(voxelizer_node::attr_ref_grid_offset, false, &stat);
			FLUID_MAYA_CHECK(stat, "plug retrieval");
			MPlug vox_ref_grid_size_plug = fn_voxelizer.findPlug(voxelizer_node::attr_ref_grid_size, false, &stat);
			FLUID_MAYA_CHECK(stat, "plug retrieval");
			MPlug vox_cells_plug = fn_voxelizer.findPlug(voxelizer_node::attr_output_cells_ref, false, &stat);
			FLUID_MAYA_CHECK(stat, "plug retrieval");

			// mesh -> voxelizer
			FLUID_MAYA_CHECK_RETURN(_graph_modifier.connect(mesh_plug, vox_mesh_plug), "attribute connection");

			// grid -> voxelizer
			FLUID_MAYA_CHECK_RETURN(
				_graph_modifier.connect(ref_cell_size_plug, vox_cell_size_plug), "attribute connection"
			);
			FLUID_MAYA_CHECK_RETURN(
				_graph_modifier.connect(ref_grid_offset_plug, vox_ref_grid_offset_plug), "attribute connection"
			);
			FLUID_MAYA_CHECK_RETURN(
				_graph_modifier.connect(ref_grid_size_plug, vox_ref_grid_size_plug), "attribute connection"
			);

			// voxelizer -> grid
			MPlug source_element = sources_plug.elementByLogicalIndex(num_sources, &stat);
			FLUID_MAYA_CHECK(stat, "attribute connection");
			source_element = source_element.child(grid_node::attr_source_cells, &stat);
			FLUID_MAYA_CHECK(stat, "attribute connection");
			FLUID_MAYA_CHECK_RETURN(
				_graph_modifier.connect(vox_cells_plug, source_element), "attribute connection"
			);
			++num_sources;
		}

		FLUID_MAYA_CHECK_RETURN(_graph_modifier.doIt(), "execute queued operations");

		return MStatus::kSuccess;
	}

	MStatus add_fluid_source_command::undoIt() {
		return _graph_modifier.undoIt();
	}

	MStatus add_fluid_source_command::redoIt() {
		return _graph_modifier.doIt();
	}
}
