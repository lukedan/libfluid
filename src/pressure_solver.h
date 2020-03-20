#pragma once

/// The pressure solver.

#include <vector>
#include <tuple>

#include "math/vec.h"
#include "fluid_grid.h"
#include "simulation.h"

namespace fluid {
	/// A pressure solver.
	class pressure_solver {
	public:
		/// Additional data for each cell.
		struct cell_data {
			/// Initializes all members to zero.
			cell_data();

			std::size_t
				nonsolid_neighbors : 3, ///< The number of neighboring solid cells. This is the diagonal term.
				fluid_xpos : 1, ///< Indicates whether the next cell in the X direction is a fluid cell.
				fluid_ypos : 1, ///< Indicates whether the next cell in the Y direction is a fluid cell.
				fluid_zpos : 1; ///< Indicates whether the next cell in the Z direction is a fluid cell.
		};

		/// Initializes this solver given the \ref fluid_grid.
		explicit pressure_solver(simulation&, const std::vector<vec3s>&);

		/// Solves for the pressure.
		///
		/// \return The pressure vector, the residual, and the number of iterations.
		[[nodiscard]] std::tuple<std::vector<double>, double, std::size_t> solve(double dt);
		/// Applies the pressure by updating the velocity field.
		void apply_pressure(double dt, const std::vector<double>&) const;

		double
			tau = 0.97, ///< The tau value.
			sigma = 0.25, ///< The sigma value.
			tolerance = 1e-6; ///< The tolerance for terminating the algorithm.
		std::size_t max_iterations = 200; ///< The maximum number of iterations.
	protected:
		/// Indicates that a cell is not a fluid cell and does not have an index in \ref _fluid_cells.
		constexpr static std::size_t _not_a_fluid_cell = std::numeric_limits<std::size_t>::max();

		/// The sparse matrix A in the book. Each element corresponds to one element in
		/// \ref fluid_grid::_fluid_cell_indices in \ref _fluid. Note that to obtain the actrual A matrix an
		/// additional coefficient needs to be multiplied.
		std::vector<cell_data> _a;
		/// The index of each cell in \ref _fluid_cells. Non-fluid cells have the value \ref _not_a_fluid_cell.
		grid3<std::size_t> _fluid_cell_indices;
		/// The velocity of solid objects.
		vec3d _usolid; // TODO remove this when implementing proper per-object velocities
		double _a_scale = 0.0; ///< The coefficient that \ref _a should be scaled by.
		/// The complete list of cells that contain fluid, sorted in the order they're stored in the grid.
		const std::vector<vec3s> &_fluid_cells;
		simulation &_sim; ///< The simulation.

		/// Returns the fluid cell index of a neighboring cell in the negative x-, y-, or z-direction.
		template <std::size_t Dim> [[nodiscard]] std::size_t _get_neg_neighbor_index(vec3s v) const {
			if (v[Dim] > 0) {
				return _fluid_cell_indices(v - vec3s::axis<Dim>());
			}
			return _not_a_fluid_cell;
		}
		/// Returns the fluid cell index of a neighboring cell in the positive x-, y-, or z-direction.
		template <std::size_t Dim> [[nodiscard]] std::size_t _get_pos_neighbor_index(vec3s v) const {
			if (v[Dim] + 1 < _sim.grid().grid().get_size()[Dim]) {
				return _fluid_cell_indices(v + vec3s::axis<Dim>());
			}
			return _not_a_fluid_cell;
		}

		/// Fills \ref _fluid_cell_indices.
		void _compute_fluid_cell_indices();

		/// Computes the matrix A.
		void _compute_a_matrix();
		/// Computes the vector b.
		[[nodiscard]] std::vector<double> _compute_b_vector() const;

		/// Computes the preconditioner vector. The input vector must have enough space (i.e., must be as long as
		/// \ref _a).
		[[nodiscard]] std::vector<double> _compute_preconditioner(double dt) const;
		/// Multiplies the given vector by the preconditioner matrix. The input vectors must have enough space.
		void _apply_preconditioner(
			std::vector<double> &z, std::vector<double> &q_scratch,
			const std::vector<double> &precon, const std::vector<double> &r
		) const;
		
		/// Multiplies the given vector by the A matrix. The out vector must have enough space.
		void _apply_a(std::vector<double>&, const std::vector<double>&) const;

		/// Calculates <tt>a + s * b</tt>. The three vectors must have the same length.
		static void _muladd(
			std::vector<double>&, const std::vector<double> &a, const std::vector<double> &b, double s
		);
	};
}
