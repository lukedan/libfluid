#pragma once

/// The pressure solver.

#include <vector>

#include "math/vec.h"
#include "fluid_grid.h"

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
		explicit pressure_solver(fluid_grid&, const std::vector<vec3s>&);

		/// Solves for the pressure.
		///
		/// \return The pressure vector and the residual.
		std::pair<std::vector<double>, double> solve(double dt);

		double
			tau = 0.97, ///< The tau value.
			sigma = 0.25, ///< The sigma value.
			tolerance = 1e-6; ///< The tolerance for terminating the algorithm.
		std::size_t max_iterations = 100; ///< The maximum number of iterations.
	protected:
		/// Indicates that a cell is not a fluid cell and does not have an index in \ref _fluid_cells.
		constexpr static std::size_t _not_a_fluid_cell = std::numeric_limits<std::size_t>::max();

		/// The sparse matrix A in the book. Each element corresponds to one element in
		/// \ref fluid_grid::_fluid_cell_indices in \ref _fluid. Note that to obtain the actrual A matrix an
		/// additional coefficient needs to be multiplied.
		std::vector<cell_data> _a;
		/// The index of each cell in \ref _fluid_cells. Non-fluid cells have the value 
		grid<std::size_t> _fluid_cell_indices;
		/// The complete list of cells that contain fluid, sorted in the order they're stored in the grid.
		const std::vector<vec3s> &_fluid_cells;
		double _a_scale = 0.0; ///< The coefficient that \ref _a should be scaled by.
		fluid_grid &_fluid; ///< The fluid grid.

		/// Returns the cell at the given index, or \p nullptr if the cell is out of bounds. Note that since unsigned
		/// overflow is well defined, "negative" coordinates works fine.
		fluid_grid::cell *_get_cell(vec3s) const;
		/// Returns the cell and its type at the given index. For cells that are outside of the simulation grid, this
		/// function returns \ref fluid_grid::cell::type::solid.
		std::pair<fluid_grid::cell*, fluid_grid::cell::type> _get_cell_and_type(vec3s) const;

		/// Computes the matrix A.
		void _compute_a_matrix();
		/// Computes the vector b.
		std::vector<double> _compute_b_vector() const;

		/// Computes the preconditioner vector. The input vector must have enough space (i.e., must be as long as
		/// \ref _a).
		std::vector<double> _compute_preconditioner(double dt) const;
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
