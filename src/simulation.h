#pragma once

/// \file
/// Complete fluid simulation.

#include <random>
#include <deque>

#include <pcg_random.hpp>

#include "misc.h"
#include "fluid_grid.h"
#include "data_structures/particle.h"
#include "data_structures/space_hashing.h"

namespace fluid {
	/// A fluid simulation.
	class simulation {
	public:
		/// The default density used when seeding particles.
		constexpr static std::size_t default_seeding_density = 2;

		/// Resizes the simulation grid.
		void resize(vec3s);

		/// Moves the simulation forward in time by the given amount.
		void update(double);
		/// Takes a single timestep using the given delta time.
		void time_step(double);

		/// Updates \ref _space_hash and \ref particle::grid_index.
		void hash_particles();

		/// Seeds the given cell so that it has at lest the given number of particles. \ref _space_hash and
		/// \ref particle::grid_index must be valid for this function to be effective. This function also updates
		/// those data accordingly.
		void seed_cell(vec3s cell, vec3d velocity, std::size_t density = default_seeding_density);

		/// Seeds the given region using the given predicate that indicates whether a point is inside the region to
		/// be seeded.
		template <typename Func> void seed_func(
			vec3s start, vec3s size, const Func &pred, std::size_t density = default_seeding_density
		) {
			double small_cell_size = cell_size / density;
			std::uniform_real_distribution<double> dist(0.0, small_cell_size);
			vec3s end(vec_ops::apply<vec3d>(
				static_cast<const double&(*)(const double&, const double&)>(std::min),
				start + size, grid().grid().get_size()
				));
			for (std::size_t z = start.z; z < end.z; ++z) {
				for (std::size_t y = start.y; y < end.y; ++y) {
					for (std::size_t x = start.x; x < end.x; ++x) {
						vec3d cell_offset = vec3d(vec3s(x, y, z)) * cell_size;
						for (std::size_t sx = 0; sx < density; ++sx) {
							for (std::size_t sy = 0; sy < density; ++sy) {
								for (std::size_t sz = 0; sz < density; ++sz) {
									vec3d position =
										grid_offset + cell_offset +
										vec3d(vec3s(sx, sy, sz)) * small_cell_size +
										vec3d(dist(random), dist(random), dist(random));
									if (pred(position)) {
										particle p;
										p.position = position;
										_particles.emplace_back(p);
									}
								}
							}
						}
					}
				}
			}
		}
		/// Seeds the simulation with water particles in the given box.
		void seed_box(vec3d start, vec3d size, std::size_t density = default_seeding_density);
		/// Seeds the simulation with water particles in the given sphere.
		void seed_sphere(vec3d center, double radius, std::size_t density = default_seeding_density);

		/// Converts a world position to a cell index, clamping it to fit in the grid.
		[[nodiscard]] vec3s world_position_to_cell_index(vec3d) const;
		/// Converts a world position to a cell index without clamping.
		[[nodiscard]] vec3s world_position_to_cell_index_unclamped(vec3d) const;

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
		[[nodiscard]] std::deque<particle> &particles() {
			return _particles;
		}
		/// \overload
		[[nodiscard]] const std::deque<particle> &particles() const {
			return _particles;
		}

		vec3d grid_offset; ///< The offset of the grid's origin.
		double cell_size = std::numeric_limits<double>::quiet_NaN(); ///< The size of each grid cell.
		double density = 1.0; ///< The density of the fluid.
		double boundary_skin_width = 0.1; ///< The "skin width" at boundaries to prevent particles from "sticking".
		pcg32 random; ///< The random engine for the simulation.
	private:
		std::deque<particle> _particles; ///< All particles.
		fluid_grid
			_grid, ///< The grid.
			_old_grid; ///< Grid that stores old velocities used for FLIP.

		/// Space hashing. This is valid after each time step, or you can call \ref hash_particles() to manually
		/// update this.
		space_hashing<particle> _space_hash;

		/// Returns the velocities of the negative direction faces of the given cell.
		[[nodiscard]] static vec3d _get_negative_face_velocities(const fluid_grid&, vec3s);
		/// Zeros the velocities at the boundaries of the grid.
		static void _remove_boundary_velocities(fluid_grid&);

		/// The kernel function used when transfering velocities to and from the grid.
		[[nodiscard]] FLUID_FORCEINLINE double _kernel(vec3d) const;

		/// Advects particles.
		void _advect_particles(double);
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
