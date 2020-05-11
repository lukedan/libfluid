#pragma once

/// \file
/// Complete fluid simulation.

#include <random>
#include <deque>
#include <functional>
#include <memory>

#include <pcg_random.hpp>

#include "misc.h"
#include "mac_grid.h"
#include "data_structures/space_hashing.h"
#include "data_structures/source.h"
#include "data_structures/grid.h"

namespace fluid {
	/// A fluid simulation.
	class simulation {
	public:
		/// A particle.
		struct particle {
			vec3d
				position, ///< The position of this particle.
				velocity, ///< The velocity of this particle.
				cx, ///< The c vector used in APIC.
				cy, ///< The c vector used in APIC.
				cz; ///< The c vector used in APIC.
			/// The final position of this particle in the *previous* time step. This is used for collision
			/// detection.
			vec3d old_position;
			std::size_t raw_cell_index = 0; ///< Raw index of the cell this particle's in.

			/// Computes the cell index of this particle. This function assumes that the computation won't result in
			/// underflow of the cell index.
			vec3s compute_cell_index(vec3d grid_offset, double cell_size) const;
			/// Computes the cell index and fraction position inside the cell of this particle. This function assumes
			/// that the computation won't result in underflow of the cell index.
			std::pair<vec3s, vec3d> compute_cell_index_and_position(vec3d grid_offset, double cell_size) const;
		};
		/// The simulation method.
		enum class method : unsigned char {
			pic, ///< PIC.
			flip_blend, ///< Linear blend between FLIP and PIC.
			apic ///< APIC.
		};

		/// If \p true, collision detection will be ran twice per time step, i.e., one extra collision detection
		/// after advection. Setting this to \p false can result in more noisy simulations.
		constexpr static bool precise_collision_detection = true;
		/// The default density used when seeding particles.
		constexpr static std::size_t default_seeding_density = 2;

		/// Resizes the simulation grid.
		void resize(vec3s);

		/// Moves the simulation forward in time by the given amount.
		void update(double);
		/// Takes a single timestep using the given delta time.
		void time_step(double);
		/// Takes a single timestep using the default CFL number.
		void time_step();

		/// Resets \ref _space_hash.
		void reset_space_hash();
		/// Updates \ref particles::raw_cell_index, then calls \ref hash_particles().
		void update_and_hash_particles();
		/// Updates \ref _space_hash. This function requires that \ref particles::raw_cell_index is valid.
		void hash_particles();

		/// Seeds the given cell so that it has at lest the given number of particles. \ref _space_hash must be valid
		/// for this function to be effective. This function updates \ref _cell_particles::count but does **not**
		/// insert the particles at the right position.
		void seed_cell(vec3s cell, vec3d velocity, std::size_t density = default_seeding_density);

		/// Seeds the given region using the given predicate that indicates whether a point is inside the region to
		/// be seeded.
		template <typename Func> void seed_func(
			vec3s start, vec3s size, const Func &pred,
			vec3d velocity = vec3d(), std::size_t density = default_seeding_density
		) {
			double small_cell_size = cell_size / density;
			std::uniform_real_distribution<double> dist(0.0, small_cell_size);
			vec3s end(vec_ops::apply<vec3s>(
				static_cast<const std::size_t & (*)(const std::size_t&, const std::size_t&)>(std::min),
				start + size, grid().grid().get_size()
				));
			for (std::size_t z = start.z; z < end.z; ++z) {
				for (std::size_t y = start.y; y < end.y; ++y) {
					for (std::size_t x = start.x; x < end.x; ++x) {
						vec3d cell_offset = vec3d(vec3s(x, y, z)) * cell_size;
						std::size_t cell_index = grid().grid().index_to_raw(vec3s(x, y, z));
						for (std::size_t sx = 0; sx < density; ++sx) {
							for (std::size_t sy = 0; sy < density; ++sy) {
								for (std::size_t sz = 0; sz < density; ++sz) {
									vec3d position =
										grid_offset + cell_offset +
										vec3d(vec3s(sx, sy, sz)) * small_cell_size +
										vec3d(dist(random), dist(random), dist(random));
									if (pred(position)) {
										particle p;
										p.old_position = p.position = position;
										p.velocity = velocity;
										p.raw_cell_index = cell_index;
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
		void seed_box(
			vec3d start, vec3d size, vec3d velocity = vec3d(), std::size_t density = default_seeding_density
		);
		/// Seeds the simulation with water particles in the given sphere.
		void seed_sphere(
			vec3d center, double radius, vec3d velocity = vec3d(), std::size_t density = default_seeding_density
		);

		/// Converts a world position to a cell index, clamping it to fit in the grid.
		[[nodiscard]] vec3s world_position_to_cell_index(vec3d) const;
		/// Converts a world position to a cell index without clamping.
		[[nodiscard]] vec3s world_position_to_cell_index_unclamped(vec3d) const;

		/// Returns the CFL condition value.
		[[nodiscard]] double cfl() const;

		/// Returns the velocity grid.
		[[nodiscard]] mac_grid &grid() {
			return _grid;
		}
		/// \overload
		[[nodiscard]] const mac_grid &grid() const {
			return _grid;
		}
		/// Returns the list of particles. Do not store references to these particles.
		[[nodiscard]] std::vector<particle> &particles() {
			return _particles;
		}
		/// \overload
		[[nodiscard]] const std::vector<particle> &particles() const {
			return _particles;
		}

		// callbacks
		// the order in which they're defined is the order in which they'll be called
		/// The function that will be called as soon as a new time step begins. The parameter is the delta time.
		std::function<void(double)> pre_time_step_callback;
		/// The function that will be called in each time step after particles have been advected. The parameter is
		/// the delta time.
		std::function<void(double)> post_advection_callback;
		/// The function that will be called in each time step after particles have been hashed and their velocities
		/// have been transferred to the grid. The parameter is the delta time.
		std::function<void(double)> post_particle_to_grid_transfer_callback;
		/// The function that will be called in each time step after gravity has been added to the velocities. The
		/// parameter is the delta time.
		std::function<void(double)> post_gravity_callback;
		/// The function that will be called in each time step after solving for pressure but before the pressure is
		/// applied. The parameter is the delta time, while the rest correspond to the return values of
		/// \ref pressure_solver::solve().
		std::function<void(double, std::vector<double>&, double, std::size_t)> post_pressure_solve_callback;
		/// The function that will be called in each time step after pressure has been applied. The parameter is the
		/// delta time.
		std::function<void(double)> post_apply_pressure_callback;
		/// The function that will be called in each time step after particle positions have been corrected. The
		/// parameter is the delta time.
		std::function<void(double)> post_correction_callback;
		/// The function that will be called in each time step after velocities have been transferred from the grid
		/// to the particles. The parameter is the delta time.
		std::function<void(double)> post_grid_to_particle_transfer_callback;

		pcg32 random; ///< The random engine for the simulation.
		std::vector<std::unique_ptr<source>> sources;
		vec3d
			grid_offset, ///< The offset of the grid's origin.
			gravity; ///< The gravity.
		double
			cfl_number = 3.0, ///< The CFL number.
			blending_factor = 1.0, ///< The factor used when blending different method together.
			cell_size = std::numeric_limits<double>::quiet_NaN(), ///< The size of each grid cell.
			density = 1.0, ///< The density of the fluid.
			boundary_skin_width = 0.1, ///< The "skin width" at boundaries to prevent particles from "sticking".
			correction_stiffness = 5.0; ///< The stiffness used when correcting particle positions.
		std::size_t velocity_extrapolation_iterations = 1; ///< The number of velocity extrapolation iterations.
		method simulation_method = method::apic; ///< The simulation method.
	private:
		/// Information about all particles in a cell.
		struct _cell_particles {
			std::size_t
				begin = 0, ///< Index of the first particle.
				count = 0; ///< The number of particles.
		};

		std::vector<particle> _particles; ///< All particles.
		mac_grid
			_grid, ///< The grid.
			_old_grid; ///< Grid that stores old velocities used for FLIP.

		/// Space hashing. This is computed by \ref hash_particles(). The way this is computed is that all particles
		/// are sorted according to \ref particle::raw_cell_index, then particles in each cell are recorded in this
		/// grid.
		grid3<_cell_particles> _space_hash;
		/// Cells that contain fluid particles. This is computed by \ref hash_particles().
		std::vector<std::size_t> _fluid_cells;

		/// Calls the callback function for all particles in the specified region.
		template <typename Cb> void _for_all_nearby_particles(
			vec3s center, vec3s diffmin, vec3s diffmax, Cb &&callback
		) {
			_space_hash.for_each_in_range_checked(
				[this, &callback](vec3s, const _cell_particles &cell) {
					for (std::size_t c = cell.begin, i = 0; i < cell.count; ++c, ++i) {
						callback(_particles[c]);
					}
				},
				center, diffmin, diffmax
					);
		}

		/// Returns the velocities of the negative direction faces of the given cell.
		[[nodiscard]] static vec3d _get_negative_face_velocities(const mac_grid&, vec3s);
		/// Zeros the velocities at the boundaries of the grid.
		static void _remove_boundary_velocities(mac_grid&);

		/// The kernel function used when transfering velocities to and from the grid. It is assumed that the input
		/// vector has already been divided by \ref cell_size.
		[[nodiscard]] FLUID_FORCEINLINE double _kernel(vec3d) const;
		/// Computes the gradient of the kernel function. It is assumed that the input vector has already been divided by
		/// \ref cell_size and has no coordinate that lies out of [-cell_size, cell_size].
		[[nodiscard]] FLUID_FORCEINLINE vec3d _grad_kernel(vec3d) const;

		/// Advects particles. Fluid sources that coerce particle velocities are processed here.
		void _advect_particles(double);

		/// Transfers velocities from particles to the grid using PIC.
		void _transfer_to_grid_pic();
		/// Transfers velocities from particles to the grid using FLIP.
		void _transfer_to_grid_flip();
		/// Transfers velocities from particles to the grid using APIC.
		void _transfer_to_grid_apic();
		/// Transfers velocities from particles to the grid using \ref simulation_method.
		void _transfer_to_grid();

		/// Transfers velocities from the grid back to particles using PIC.
		void _transfer_from_grid_pic();
		/// Transfers velocities from the grid back to particles using a blend between PIC and FLIP.
		///
		/// \param blend The blend factor. 1.0 means fully FLIP.
		void _transfer_from_grid_flip(double blend);
		/// Calculates the c vector.
		[[nodiscard]] vec3d _calculate_c_vector(
			double v000, double v001, double v010, double v011,
			double v100, double v101, double v110, double v111,
			double tx, double ty, double tz
		) const;
		/// Transfers velocities from the grid back to particles using APIC.
		void _transfer_from_grid_apic();
		/// Transfers velocities from the grid back to particles using \ref simulation_method.
		void _transfer_from_grid();

		/// Adds spring forces between particles to reduce clumping. \ref _space_hash must have been filled before
		/// this is called. This is taken from "Preserving Fluid Sheets with Adaptively Sampled Anisotropic
		/// Particles". After this function returns, \ref particle::raw_cell_index and \ref _space_hash are **not**
		/// valid.
		void _correct_positions(double dt);

		/// Detects collisions.
		void _detect_collisions();

		/// Velocity extrapolation.
		void _extrapolate_velocities(const std::vector<vec3s> &fluid_cells);

		/// Seeds all fluid sources.
		void _update_sources();
	};
}
