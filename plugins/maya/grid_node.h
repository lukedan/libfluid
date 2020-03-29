#pragma once

/// \file
/// Declaration of the node that represents the simulation grid.

#include <vector>
#include <deque>

#include <maya/MPxNode.h>
#include <maya/MPointArray.h>

#include <fluid/data_structures/particle.h>

namespace fluid::maya {
	/// A node that represents the simulation grid.
	class grid_node : public MPxNode {
	public:
		static MObject
			attr_time, ///< The time attribute.
			attr_cell_size, ///< The cell size attribute.
			attr_grid_size, ///< The grid size attribute.
			attr_grid_offset, ///< The grid offset attribute.
			attr_gravity, ///< The gravity attribute.
			attr_transfer_method, ///< The transfer method attribute.
			attr_output_particle_positions; ///< The particle positions attribute.
		static MTypeId id; ///< The ID of this node.

		/// Updates the simulation.
		MStatus compute(const MPlug&, MDataBlock&) override;

		/// When input attributes other than time is changed, clear all cached data.
		MStatus setDependentsDirty(const MPlug&, MPlugArray&) override;

		/// Creation function passed to \p MFnPlugin::registerNode().
		static void *creator();
		/// Initialization function passed to \p MFnPlugin::registerNode().
		static MStatus initialize();
	private:
		// deque to reduce move operations
		std::deque<MPointArray> _particle_cache; ///< Cached particle positions for each frame.
		std::deque<fluid::particle> _last_frame_particles; ///< Saved particles from the last cached frame.
	};
}
