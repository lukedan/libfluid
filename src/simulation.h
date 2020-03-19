#pragma once

/// \file
/// Complete fluid simulation.

#include <pcg_random.hpp>

#include "misc.h"
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
		[[nodiscard]] std::vector<particle> &particles() {
			return _particles;
		}
		/// \overload
		[[nodiscard]] const std::vector<particle> &particles() const {
			return _particles;
		}

		vec3d grid_offset; ///< The offset of the grid's origin.
		double cell_size = std::numeric_limits<double>::quiet_NaN(); ///< The size of each grid cell.
		double density = 1.0; ///< The density of the fluid.
		double boundary_skin_width = 0.1; ///< The "skin width" at boundaries to prevent particles from "sticking".
		pcg32 random; ///< The random engine for the simulation.
	private:
		std::vector<particle> _particles; ///< All particles.
		fluid_grid
			_grid, ///< The grid.
			_old_grid; ///< Grid that stores old velocities used for FLIP.

		space_hashing<particle> _space_hash; ///< Space hashing.

		/// Returns the velocities of the negative direction faces of the given cell.
		[[nodiscard]] static vec3d _get_negative_face_velocities(const fluid_grid&, vec3s);
		/// Zeros the velocities at the boundaries of the grid.
		static void _remove_boundary_velocities(fluid_grid&);

		/// The kernel function used when transfering velocities to and from the grid.
		[[nodiscard]] FLUID_FORCEINLINE double _kernel(vec3d) const;

		/// Advects particles.
		void _advect_particles(double);
		/// Updates \ref _space_hash.
		void _hash_particles();
		/// Transfers velocities from particles to the grid.
		void _transfer_to_grid_pic_flip();
		/// Transfers velocities from the grid back to particles using PIC.
		void _transfer_from_grid_pic();
		/// Transfers velocities from the grid back to particles using a blend between PIC and FLIP.
		///
		/// \param blend The blend factor. 1.0 means fully FLIP.
		void _transfer_from_grid_flip(double blend);

		/// Adds spring forces between particles to reduce clumping. \ref _space_hash must have been filled before
		/// this is called. This is taken from https://github.com/nepluno/apic2d.
		void _add_spring_forces(double dt, std::size_t step, std::size_t substep);
	};
}
