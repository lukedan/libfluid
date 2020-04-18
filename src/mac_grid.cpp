#include "fluid/mac_grid.h"

/// \file
/// Implementation of the fluid grid.

namespace fluid {
	mac_grid::mac_grid(vec3s grid_count) : _grid(grid_count) {
	}

	mac_grid::cell *mac_grid::get_cell(vec3s i) {
		vec3s grid_size = grid().get_size();
		if (i.x >= grid_size.x || i.y >= grid_size.y || i.z >= grid_size.z) {
			return nullptr;
		}
		return &grid()(i);
	}

	const mac_grid::cell *mac_grid::get_cell(vec3s i) const {
		vec3s grid_size = grid().get_size();
		if (i.x >= grid_size.x || i.y >= grid_size.y || i.z >= grid_size.z) {
			return nullptr;
		}
		return &grid()(i);
	}

	std::pair<mac_grid::cell*, mac_grid::cell::type> mac_grid::get_cell_and_type(vec3s i) {
		if (cell *cell = get_cell(i)) {
			return { cell, cell->cell_type };
		}
		return { nullptr, cell::type::solid };
	}

	std::pair<const mac_grid::cell*, mac_grid::cell::type> mac_grid::get_cell_and_type(vec3s i) const {
		if (const cell *c = get_cell(i)) {
			return { c, c->cell_type };
		}
		return { nullptr, cell::type::solid };
	}

	/// Clamps the input value, returning both the result and whether the value has been modified. Note that this
	/// function returns clamped for the max value, so that velocities at the max borders are ignored.
	std::pair<std::size_t, bool> _clamp(std::size_t val, std::size_t min, std::size_t max) {
		if (val < min) {
			return { min, true };
		}
		if (val >= max) {
			return { max, true };
		}
		return { val, false };
	}
	std::pair<mac_grid::face_samples, vec3d> mac_grid::get_face_samples(
		vec3s grid_index, vec3d offset
	) const {
		//         z  y  x
		vec3d vels[3][3][3];
		for (std::size_t dz = 0; dz < 3; ++dz) {
			auto [cz, zclamp] = _clamp(grid_index.z + dz, 1, grid().get_size().z);
			--cz;
			for (std::size_t dy = 0; dy < 3; ++dy) {
				auto [cy, yclamp] = _clamp(grid_index.y + dy, 1, grid().get_size().y);
				--cy;
				for (std::size_t dx = 0; dx < 3; ++dx) {
					auto [cx, xclamp] = _clamp(grid_index.x + dx, 1, grid().get_size().x);
					--cx;

					vec3d vel = grid()(cx, cy, cz).velocities_posface;
					if (xclamp) {
						vel.x = 0.0;
					}
					if (yclamp) {
						vel.y = 0.0;
					}
					if (zclamp) {
						vel.z = 0.0;
					}
					vels[dz][dy][dx] = vel;
				}
			}
		}
		std::size_t dx = 1, dy = 1, dz = 1;
		vec3d tmid = offset - vec3d(0.5, 0.5, 0.5);
		if (tmid.x < 0.0) {
			dx = 0;
			tmid.x += 1.0;
		}
		if (tmid.y < 0.0) {
			dy = 0;
			tmid.y += 1.0;
		}
		if (tmid.z < 0.0) {
			dz = 0;
			tmid.z += 1.0;
		}
		face_samples result;
		// v000(vels[dz    ][dy    ][0].x, vels[dz    ][0][dx    ].y, vels[0][dy    ][dx    ].z)
		// v001(vels[dz    ][dy    ][1].x, vels[dz    ][0][dx + 1].y, vels[0][dy    ][dx + 1].z)
		// v010(vels[dz    ][dy + 1][0].x, vels[dz    ][1][dx    ].y, vels[0][dy + 1][dx    ].z)
		// v011(vels[dz    ][dy + 1][1].x, vels[dz    ][1][dx + 1].y, vels[0][dy + 1][dx + 1].z)
		// v100(vels[dz + 1][dy    ][0].x, vels[dz + 1][0][dx    ].y, vels[1][dy    ][dx    ].z)
		// v101(vels[dz + 1][dy    ][1].x, vels[dz + 1][0][dx + 1].y, vels[1][dy    ][dx + 1].z)
		// v110(vels[dz + 1][dy + 1][0].x, vels[dz + 1][1][dx    ].y, vels[1][dy + 1][dx    ].z)
		// v111(vels[dz + 1][dy + 1][1].x, vels[dz + 1][1][dx + 1].y, vels[1][dy + 1][dx + 1].z)
		result.v000 = vec3d(vels[dz][dy][0].x, vels[dz][0][dx].y, vels[0][dy][dx].z);
		result.v001 = vec3d(vels[dz][dy][1].x, vels[dz][0][dx + 1].y, vels[0][dy][dx + 1].z);
		result.v010 = vec3d(vels[dz][dy + 1][0].x, vels[dz][1][dx].y, vels[0][dy + 1][dx].z);
		result.v011 = vec3d(vels[dz][dy + 1][1].x, vels[dz][1][dx + 1].y, vels[0][dy + 1][dx + 1].z);
		result.v100 = vec3d(vels[dz + 1][dy][0].x, vels[dz + 1][0][dx].y, vels[1][dy][dx].z);
		result.v101 = vec3d(vels[dz + 1][dy][1].x, vels[dz + 1][0][dx + 1].y, vels[1][dy][dx + 1].z);
		result.v110 = vec3d(vels[dz + 1][dy + 1][0].x, vels[dz + 1][1][dx].y, vels[1][dy + 1][dx].z);
		result.v111 = vec3d(vels[dz + 1][dy + 1][1].x, vels[dz + 1][1][dx + 1].y, vels[1][dy + 1][dx + 1].z);
		return { result, tmid };
	}
}
