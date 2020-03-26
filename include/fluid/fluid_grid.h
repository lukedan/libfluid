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
		/// Stores velocity samples on faces near a particle. The coordinates of each member does not necessarily
		/// correspond to that of the same cell.
		struct face_samples {
			vec3d
				v000, ///< Velocity at (0, 0, 0).
				v001, ///< Velocity at (1, 0, 0).
				v010, ///< Velocity at (0, 1, 0).
				v011, ///< Velocity at (1, 1, 0).
				v100, ///< Velocity at (0, 0, 1).
				v101, ///< Velocity at (1, 0, 1).
				v110, ///< Velocity at (0, 1, 1).
				v111; ///< Velocity at (1, 1, 1).
		};

		/// Default constructor.
		fluid_grid() = default;
		/// Initializes the grid.
		explicit fluid_grid(vec3s grid_count);

		/// Returns the velocities around the given position. Velocities of cells that are out of the grid have their
		/// corresponding coordinates clamped to zero.
		[[nodiscard]] std::pair<face_samples, vec3d> get_face_samples(vec3s grid_index, vec3d offset) const;

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
