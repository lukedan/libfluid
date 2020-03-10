#pragma once

/// \file
/// A MAC grid that stores fluid information including density and velocity.

#include "data_structures/grid.h"

namespace fluid {
	class pressure_solver;

	/// Stores the simuation grid.
	class fluid_grid {
		friend pressure_solver;
	public:
		/// A cell in the simulation grid.
		struct cell {
			/// The type of this cell's contents.
			enum class type : unsigned char {
				air = 0x1, ///< The cell contains air.
				fluid = 0x2, ///< The cell contains fluid.
				solid = 0x4, ///< The cell contains solid.
			};

			vec3d velocities_posface; ///< The velocities at positive directions.
			type cell_type = type::air; ///< The contents of this cell.
		};

		/// Initializes the grid.
		fluid_grid(double cell_size, vec3s grid_count);

		/// Returns the simulation grid.
		const grid<cell> &get_grid() const {
			return _grid;
		}

		/// Returns the size of a cell.
		double get_cell_size() const {
			return _cell_size;
		}

		/// Returns the fluid density.
		double get_density() const {
			return _density;
		}
	protected:
		grid<cell> _grid; ///< The simulation grid.
		double _cell_size = std::numeric_limits<double>::quiet_NaN(); ///< The size of each grid cell.
		double _density = 1.0; ///< The density of the fluid.
	};
}
