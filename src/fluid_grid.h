#pragma once

/// \file
/// A MAC grid that stores fluid information including density and velocity.

#include "data_structures/grid.h"

namespace fluid {
	class pressure_solver;

	/// Stores the simuation grid.
	class fluid_grid {
	public:
		/// A cell in the simulation grid.
		struct cell {
			/// The type of this cell's contents.
			enum class type : unsigned char {
				air = 0x1, ///< The cell contains air.
				fluid = 0x2, ///< The cell contains fluid.
				solid = 0x4, ///< The cell contains solid.
			};

			/// The velocities sampled at the centers of faces in the positive direction. Or, if this cell is part of
			/// a fluid source, then this is the velocity of that source.
			vec3d velocities_posface;
			type cell_type = type::air; ///< The contents of this cell.
		};

		/// Default constructor.
		fluid_grid() = default;
		/// Initializes the grid.
		explicit fluid_grid(vec3s grid_count);

		/// Returns the cell at the given index, or \p nullptr if the cell is out of bounds. Note that since unsigned
		/// overflow is well defined, "negative" coordinates works fine.
		[[nodiscard]] fluid_grid::cell *get_cell(vec3s);
		/// \overload
		[[nodiscard]] const fluid_grid::cell *get_cell(vec3s) const;

		/// Returns the cell and its type at the given index. For cells that are outside of the simulation grid, this
		/// function returns \ref fluid_grid::cell::type::solid.
		[[nodiscard]] std::pair<fluid_grid::cell*, fluid_grid::cell::type> get_cell_and_type(vec3s);
		/// \overload
		[[nodiscard]] std::pair<const fluid_grid::cell*, fluid_grid::cell::type> get_cell_and_type(vec3s) const;

		/// Returns the simulation grid.
		[[nodiscard]] grid3<cell> &grid() {
			return _grid;
		}
		/// Returns the simulation grid.
		[[nodiscard]] const grid3<cell> &grid() const {
			return _grid;
		}
	protected:
		grid3<cell> _grid; ///< The simulation grid.
	};
}
