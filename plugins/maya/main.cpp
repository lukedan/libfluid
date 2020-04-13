/// \file
/// Contains entry points of the plugin.

#include <maya/MStatus.h>
#include <maya/MObject.h>
#include <maya/MFnPlugin.h>
#include <maya/MStringArray.h>
#include <maya/MGlobal.h>

#include "misc.h"
#include "nodes/grid_node.h"
#include "nodes/mesher_node.h"
#include "nodes/point_cloud_loader_node.h"
#include "nodes/voxelizer_node.h"
#include "commands/create_simulation_grid.h"
#include "commands/add_fluid_source.h"

namespace fluid::maya {
	MTypeId
		grid_node::id{ 0x98765432 },
		mesher_node::id{ 0x98765433 },
		point_cloud_loader_node::id{ 0x98765434 },
		voxelizer_node::id{ 0x98765435 };

	const MString
		create_simulation_grid_command::name{ "fluidCreateSimulationGrid" },
		add_fluid_source_command::name{ "fluidAddFluidSource" };
}

const MString top_level_menu_name{ "libfluid" };
const MString create_simulation_grid_menu_name{ "Create Simulation Grid" };
const MString add_fluid_source_menu_name{ "Add as Fluid Source" };

MStringArray create_simulation_grid_menu;
MStringArray add_fluid_source_menu;

/// Initializes the plugin.
MStatus initializePlugin(MObject obj) {
	MStatus stat = MStatus::kSuccess;
	MFnPlugin plugin(obj, "Xuanyi Zhou", "0.1", "Any", &stat);
	FLUID_MAYA_CHECK(stat, "plugin creation");

	FLUID_MAYA_CHECK_RETURN(
		plugin.registerNode(
			"MesherNode", fluid::maya::mesher_node::id,
			fluid::maya::mesher_node::creator, fluid::maya::mesher_node::initialize
		),
		"node registration"
	);
	FLUID_MAYA_CHECK_RETURN(
		plugin.registerNode(
			"PointCloudLoaderNode", fluid::maya::point_cloud_loader_node::id,
			fluid::maya::point_cloud_loader_node::creator, fluid::maya::point_cloud_loader_node::initialize
		),
		"node registration"
	);
	FLUID_MAYA_CHECK_RETURN(
		plugin.registerNode(
			"GridNode", fluid::maya::grid_node::id,
			fluid::maya::grid_node::creator, fluid::maya::grid_node::initialize
		),
		"node registration"
	);
	FLUID_MAYA_CHECK_RETURN(
		plugin.registerNode(
			"VoxelizerNode", fluid::maya::voxelizer_node::id,
			fluid::maya::voxelizer_node::creator, fluid::maya::voxelizer_node::initialize
		),
		"node registration"
	);

	FLUID_MAYA_CHECK_RETURN(
		plugin.registerCommand(
			fluid::maya::create_simulation_grid_command::name,
			fluid::maya::create_simulation_grid_command::creator
		),
		"command registration"
	);
	FLUID_MAYA_CHECK_RETURN(
		plugin.registerCommand(
			fluid::maya::add_fluid_source_command::name,
			fluid::maya::add_fluid_source_command::creator
		),
		"command registration"
	);


	MString top_level_menu_path;
	FLUID_MAYA_CHECK_RETURN(
		MGlobal::executeCommand("menu -parent MayaWindow -label " + top_level_menu_name + ";", top_level_menu_path),
		"top level menu registration"
	);

	create_simulation_grid_menu = plugin.addMenuItem(
		create_simulation_grid_menu_name, top_level_menu_path,
		fluid::maya::create_simulation_grid_command::name, "",
		false, nullptr, &stat
	);
	FLUID_MAYA_CHECK(stat, "menu item registration");

	add_fluid_source_menu = plugin.addMenuItem(
		add_fluid_source_menu_name, top_level_menu_path,
		fluid::maya::add_fluid_source_command::name, "",
		false, nullptr, &stat
	);
	FLUID_MAYA_CHECK(stat, "menu item registration");

	return MStatus::kSuccess;
}

/// Uninitializes the plugin.
MStatus uninitializePlugin(MObject obj) {
	MFnPlugin plugin(obj);

	FLUID_MAYA_CHECK_RETURN(plugin.deregisterNode(fluid::maya::mesher_node::id), "node deregisteration");
	FLUID_MAYA_CHECK_RETURN(plugin.deregisterNode(fluid::maya::point_cloud_loader_node::id), "node deregisteration");
	FLUID_MAYA_CHECK_RETURN(plugin.deregisterNode(fluid::maya::grid_node::id), "node deregistration");
	FLUID_MAYA_CHECK_RETURN(plugin.deregisterNode(fluid::maya::voxelizer_node::id), "node deregistration");

	FLUID_MAYA_CHECK_RETURN(
		plugin.deregisterCommand(fluid::maya::create_simulation_grid_command::name), "command deregisteration"
	);
	FLUID_MAYA_CHECK_RETURN(
		plugin.deregisterCommand(fluid::maya::add_fluid_source_command::name), "command deregisteration"
	);

	FLUID_MAYA_CHECK_RETURN(plugin.removeMenuItem(create_simulation_grid_menu), "menu item deregistration");
	FLUID_MAYA_CHECK_RETURN(plugin.removeMenuItem(add_fluid_source_menu), "menu item deregistration");

	return MStatus::kSuccess;
}
