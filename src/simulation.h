#pragma once

/// \file
/// Complete fluid simulation.

#include "data_structures/fluid_grid.h"
#include "data_structures/particle.h"
#include "data_structures/space_hashing.h"

namespace fluid {
	/// A fluid simulation.
	class simulation {
	public:
		/// Resizes the simulation grid.
		void resize(vec3s);

		/// Moves the simulation forward in time by the given amount.
		void update(double);
		/// Takes a single timestep using the given delta time.
		void time_step(double);

		/// Seeds the simulation with water particles in the given rectangular region.
		void seed_box(vec3s start, vec3s size, std::size_t density = 2);

		/// Returns the CFL condition value.
		[[nodiscard]] double cfl() const;

		/// Returns the velocity grid.
		[[nodiscard]] fluid_grid &grid() {
			return _grid;
		}
		/// \overload
		[[nodiscard]] const fluid_grid &grid() const {
			return _grid;
		}
		/// Returns the list of particles.
		[[nodiscard]] const std::vector<particle> &get_particles() const {
			return _particles;
		}

		vec3d grid_offset; ///< The offset of the grid's origin.
		double cell_size = std::numeric_limits<double>::quiet_NaN(); ///< The size of each grid cell.
		double density = 1.0; ///< The density of the fluid.
	private:
		std::vector<particle> _particles; ///< All particles.
		fluid_grid _grid; ///< The grid.

		space_hashing<particle> _space_hash; ///< Space hashing.

		/// The kernel function used when transfering velocities to and from the grid.
		[[nodiscard]] double _kernel(vec3d) const;

		/// Advects particles.
		void _advect_particles(double);
		/// Updates \ref _space_hash.
		void _hash_particles();
		/// Transfers velocities from particles to the grid.
		void _transfer_to_grid();
		/// Transfers velocities from the grid back to particles.
		void _transfer_from_grid();
	};
}
