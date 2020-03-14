#include "fluid_grid.h"

/// \file
/// Implementation of the fluid grid.

namespace fluid {
	fluid_grid::fluid_grid(vec3s grid_count) : _grid(grid_count) {
	}

	fluid_grid::cell *fluid_grid::get_cell(vec3s i) {
		vec3s grid_size = grid().get_size();
		if (i.x >= grid_size.x || i.y >= grid_size.y || i.z >= grid_size.z) {
			return nullptr;
		}
		return &grid()(i);
	}

	const fluid_grid::cell *fluid_grid::get_cell(vec3s i) const {
		vec3s grid_size = grid().get_size();
		if (i.x >= grid_size.x || i.y >= grid_size.y || i.z >= grid_size.z) {
			return nullptr;
		}
		return &grid()(i);
	}

	std::pair<fluid_grid::cell*, fluid_grid::cell::type> fluid_grid::get_cell_and_type(vec3s i) {
		if (cell *cell = get_cell(i)) {
			return { cell, cell->cell_type };
		}
		return { nullptr, cell::type::solid };
	}

	std::pair<const fluid_grid::cell*, fluid_grid::cell::type> fluid_grid::get_cell_and_type(vec3s i) const {
		if (const cell *c = get_cell(i)) {
			return { c, c->cell_type };
		}
		return { nullptr, cell::type::solid };
	}
}
