#pragma once

/// \file
/// Declaration of the node that represents the simulation grid.

#include <vector>
#include <deque>

#include <maya/MPxNode.h>
#include <maya/MPxLocatorNode.h>
#include <maya/MPointArray.h>
#include <maya/MPxDrawOverride.h>

#include <fluid/simulation.h>

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
			// fluid source attributes
			attr_source_cells, ///< The cells occupied by each fluid source.
			attr_source_velocity, ///< The velocities of each fluid source.
			attr_source_enabled, ///< Whether each fluid source is enabled.
			/// Whether or not to force all particles in each fluid source to have the desired velocity.
			attr_source_coerce_velocity,
			attr_source_seeding_density, ///< The desired density inside the source region.
			attr_sources, ///< Compound attribute that includes data of all fluid sources.
			// obstacle attributes
			attr_obstacle_cells, ///< The cells that belong to a obstacle.
			attr_obstacles, ///< Compound attribute that includes all obstacles.
			// output
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
		std::vector<simulation::particle> _last_frame_particles; ///< Saved particles from the last cached frame.
	};
}
