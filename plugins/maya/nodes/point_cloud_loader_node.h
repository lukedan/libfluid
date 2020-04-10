#pragma once

/// \file
/// Declaration of the point cloud loader node.

#include <maya/MPxNode.h>

namespace fluid::maya {
	/// The point cloud loader node.
	class point_cloud_loader_node : public MPxNode {
	public:
		static MObject
			attr_file_name, ///< The file name attribute.
			attr_output_points; ///< The output point cloud attribute.
		static MTypeId id; ///< The ID of this node.

		/// Reloads the point cloud.
		MStatus compute(const MPlug&, MDataBlock&) override;

		/// Creation function passed to \p MFnPlugin::registerNode().
		static void *creator();
		/// Initialization function passed to \p MFnPlugin::registerNode().
		static MStatus initialize();
	};
}
