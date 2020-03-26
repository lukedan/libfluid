/// \file
/// Contains entry points of the plugin.

#include <maya/MStatus.h>
#include <maya/MObject.h>
#include <maya/MFnPlugin.h>

#include "misc.h"
#include "mesher_node.h"
#include "point_cloud_loader_node.h"

/// Initializes the plugin.
MStatus initializePlugin(MObject obj) {
	MStatus status = MStatus::kSuccess;
	MFnPlugin plugin(obj, "Xuanyi Zhou", "0.1", "Any", &status);
	FLUID_MAYA_CHECK(status, "plugin creation");

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

	return status;
}

/// Uninitializes the plugin.
MStatus uninitializePlugin(MObject obj) {
	MStatus status = MStatus::kSuccess;
	MFnPlugin plugin(obj);

	FLUID_MAYA_CHECK_RETURN(plugin.deregisterNode(fluid::maya::mesher_node::id), "node deregisteration");
	FLUID_MAYA_CHECK_RETURN(plugin.deregisterNode(fluid::maya::point_cloud_loader_node::id), "node deregisteration");

	return status;
}
