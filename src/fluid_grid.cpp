#include "fluid_grid.h"

/// \file
/// Implementation of the fluid grid.

namespace fluid {
	fluid_grid::fluid_grid(double cell_size, vec3s grid_count) : _grid(grid_count), _cell_size(cell_size) {
	}
}
