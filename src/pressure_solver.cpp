#include "pressure_solver.h"

/// \file
/// Implementation of the pressure solver.

#include <algorithm>

namespace fluid {
	pressure_solver::cell_data::cell_data() : nonsolid_neighbors(0), fluid_xpos(0), fluid_ypos(0), fluid_zpos(0) {
	}


	pressure_solver::pressure_solver(fluid_grid &grid, const std::vector<vec3s> &fluid_cells) :
		_fluid_cell_indices(grid._grid.get_size(), _not_a_fluid_cell), _fluid(grid), _fluid_cells(fluid_cells) {
	}

	fluid_grid::cell *pressure_solver::_get_cell(vec3s i) const {
		vec3s grid_size = _fluid._grid.get_size();
		if (i.x >= grid_size.x || i.y >= grid_size.y || i.z >= grid_size.z) {
			return nullptr;
		}
		return &_fluid._grid(i);
	}

	std::pair<fluid_grid::cell*, fluid_grid::cell::type> pressure_solver::_get_cell_and_type(vec3s i) const {
		if (fluid_grid::cell *cell = _get_cell(i)) {
			return { cell, cell->cell_type };
		}
		return { nullptr, fluid_grid::cell::type::solid };
	}

	std::pair<std::vector<double>, double> pressure_solver::solve(double dt) {
		_a_scale = dt / (_fluid.get_density() * _fluid.get_cell_size() * _fluid.get_cell_size());
		_compute_a_matrix();
		std::vector<double> b = _compute_b_vector();
		std::vector<double> precon = _compute_preconditioner(dt);
		double residual = 0.0;

		std::vector<double> p(_fluid_cells.size(), 0.0), r = b;

		std::vector<double> z(_fluid_cells.size(), 0.0), q_scratch(_fluid_cells.size(), 0.0);
		_apply_preconditioner(z, q_scratch, precon, r);
		std::vector<double> s = z;

		double sigma = vec_ops::dot(z, r);

		for (std::size_t i = 0; i < max_iterations; ++i) {

			_apply_a(z, s);

			double alpha = sigma / vec_ops::dot(z, s);

			_muladd(p, p, s, alpha);
			_muladd(r, r, z, -alpha);

			residual = *std::max_element(r.begin(), r.end());
			if (residual < tolerance) {
				break;
			}

			_apply_preconditioner(z, q_scratch, precon, r);

			double sigma_new = vec_ops::dot(z, r);

			double beta = sigma_new / sigma;

			_muladd(s, z, s, beta);

			sigma = sigma_new;
		}
		return { p, residual };
	}

	const std::vector<vec3s> _offsets{
		{ 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 }, { -1, 0, 0 }, { 0, -1, 0 }, { 0, 0, -1 }
	};
	void pressure_solver::_compute_a_matrix() {
		_a.reserve(_fluid_cells.size());
		for (std::size_t i = 0; i < _fluid_cells.size(); ++i) { // border cells
			cell_data data;
			vec3s pos = _fluid_cells[i];
			for (vec3s o : _offsets) {
				data.nonsolid_neighbors +=
					_get_cell_and_type(pos + o).second != fluid_grid::cell::type::solid ? 1 : 0;
			}
			data.fluid_xpos =
				_get_cell_and_type(pos + vec3s::axis<0>()).second == fluid_grid::cell::type::fluid ? 1 : 0;
			data.fluid_ypos =
				_get_cell_and_type(pos + vec3s::axis<1>()).second == fluid_grid::cell::type::fluid ? 1 : 0;
			data.fluid_zpos =
				_get_cell_and_type(pos + vec3s::axis<2>()).second == fluid_grid::cell::type::fluid ? 1 : 0;
			_a.emplace_back(data);
		}
	}

	std::vector<double> pressure_solver::_compute_b_vector() const {
		std::vector<double> b(_fluid_cells.size(), 0.0);
		double scale = 1.0 / _fluid.get_cell_size();
		vec3d usolid; // TODO zero solid velocity
		for (std::size_t i = 0; i < _fluid_cells.size(); ++i) {
			vec3s pos = _fluid_cells[i];
			vec3d vel = _fluid._grid(pos).velocities_posface;

			double value = -(vel.x + vel.y + vel.z);

			if (pos.x > 0) {
				const fluid_grid::cell &cell = _fluid._grid(pos.x - 1, pos.y, pos.z);
				value += cell.velocities_posface.x;
				if (cell.cell_type == fluid_grid::cell::type::solid) {
					value += usolid.x - cell.velocities_posface.x;
				}
			} else {
				value += usolid.y;
			}
			if (pos.y > 0) {
				const fluid_grid::cell &cell = _fluid._grid(pos.x, pos.y - 1, pos.z);
				value += cell.velocities_posface.y;
				if (cell.cell_type == fluid_grid::cell::type::solid) {
					value += usolid.y - cell.velocities_posface.y;
				}
			} else {
				value += usolid.x;
			}
			if (pos.z > 0) {
				const fluid_grid::cell &cell = _fluid._grid(pos.x, pos.y, pos.z - 1);
				value += cell.velocities_posface.z;
				if (cell.cell_type == fluid_grid::cell::type::solid) {
					value += usolid.z - cell.velocities_posface.z;
				}
			} else {
				value += usolid.z;
			}

			if (_get_cell_and_type(pos + vec3s::axis<0>()).second == fluid_grid::cell::type::solid) {
				value += vel.x - usolid.x;
			}
			if (_get_cell_and_type(pos + vec3s::axis<1>()).second == fluid_grid::cell::type::solid) {
				value += vel.y - usolid.y;
			}
			if (_get_cell_and_type(pos + vec3s::axis<2>()).second == fluid_grid::cell::type::solid) {
				value += vel.z - usolid.z;
			}

			b[i] = scale * value;
		}
		return b;
	}

	std::vector<double> pressure_solver::_compute_preconditioner(double dt) const {
		std::vector<double> precon(_fluid_cells.size(), 0.0);
		for (std::size_t i = 0; i < _fluid_cells.size(); ++i) {
			vec3s pos = _fluid_cells[i];
			double
				neg_e = 0.0, // the negative part of e without tau that needs to be scaled by _a_scale^2
				neg_e_tau = 0.0; // the negative part of e with tau that needs to be scaled by tau * _a_scale^2

			if (pos.x > 0) {
				std::size_t xneg_index = _fluid_cell_indices(pos.x - 1, pos.y, pos.z);
				if (xneg_index != _not_a_fluid_cell) {
					const cell_data &xneg_cell = _a[xneg_index];
					double
						xneg_precon = precon[xneg_index],
						a_times_precon = xneg_cell.fluid_xpos * xneg_precon;
					neg_e += a_times_precon * a_times_precon;
					neg_e_tau +=
						(xneg_cell.fluid_xpos * (xneg_cell.fluid_ypos + xneg_cell.fluid_zpos)) *
						xneg_precon * xneg_precon;
				}
			}

			if (pos.y > 0) {
				std::size_t yneg_index = _fluid_cell_indices(pos.x, pos.y - 1, pos.z);
				if (yneg_index != _not_a_fluid_cell) {
					const cell_data &yneg_cell = _a[yneg_index];
					double
						yneg_precon = precon[yneg_index],
						a_times_precon = yneg_cell.fluid_ypos * yneg_precon;
					neg_e += a_times_precon * a_times_precon;
					neg_e_tau +=
						(yneg_cell.fluid_ypos * (yneg_cell.fluid_xpos + yneg_cell.fluid_zpos)) *
						yneg_precon * yneg_precon;
				}
			}

			if (pos.z > 0) {
				std::size_t zneg_index = _fluid_cell_indices(pos.x, pos.y, pos.z - 1);
				if (zneg_index != _not_a_fluid_cell) {
					const cell_data &zneg_cell = _a[zneg_index];
					double
						zneg_precon = precon[zneg_index],
						a_times_precon = zneg_cell.fluid_zpos * zneg_precon;
					neg_e += a_times_precon * a_times_precon;
					neg_e_tau +=
						(zneg_cell.fluid_zpos * (zneg_cell.fluid_xpos + zneg_cell.fluid_ypos)) *
						zneg_precon * zneg_precon;
				}
			}

			// still needs to be scaled by _a_scale later
			double e = _a[i].nonsolid_neighbors - (neg_e + tau * neg_e_tau) * _a_scale;

			if (e < sigma * _a[i].nonsolid_neighbors) {
				e = _a[i].nonsolid_neighbors;
			}
			precon[i] = 1.0 / std::sqrt(e * _a_scale);
		}
		return precon;
	}

	void pressure_solver::_apply_preconditioner(
		std::vector<double> &z, std::vector<double> &q_scratch,
		const std::vector<double> &precon, const std::vector<double> &r
	) const {
		// first solve L q = r
		for (std::size_t i = 0; i < _fluid_cells.size(); ++i) {
			vec3s pos = _fluid_cells[i];
			double neg_t = 0.0; // the negative part of t that needs to be scaled by _a_scale
			if (pos.x > 0) {
				std::size_t xneg_index = _fluid_cell_indices(pos.x - 1, pos.y, pos.z);
				if (xneg_index != _not_a_fluid_cell) {
					neg_t += _a[xneg_index].fluid_xpos * precon[xneg_index] * q_scratch[xneg_index];
				}
			}
			if (pos.y > 0) {
				std::size_t yneg_index = _fluid_cell_indices(pos.x, pos.y - 1, pos.z);
				if (yneg_index != _not_a_fluid_cell) {
					neg_t += _a[yneg_index].fluid_ypos * precon[yneg_index] * q_scratch[yneg_index];
				}
			}
			if (pos.z > 0) {
				std::size_t zneg_index = _fluid_cell_indices(pos.x, pos.y, pos.z - 1);
				if (zneg_index != _not_a_fluid_cell) {
					neg_t += _a[zneg_index].fluid_zpos * precon[zneg_index] * q_scratch[zneg_index];
				}
			}
			q_scratch[i] = (r[i] - _a_scale * neg_t) * precon[i];
		}
		// next solve L^T z = q
		for (std::size_t i = _fluid_cells.size(); i > 0; ) {
			--i;
			vec3s pos = _fluid_cells[i];
			const cell_data &cell = _a[i];
			double neg_t = 0.0; // the negative part of t that needs to be scaled by _a_scale * precon[i]
			if (pos.x + 1 < _fluid._grid.get_size().x) {
				std::size_t xpos_index = _fluid_cell_indices(pos.x + 1, pos.y, pos.z);
				if (xpos_index != _not_a_fluid_cell) {
					neg_t += cell.fluid_xpos * z[xpos_index];
				}
			}
			if (pos.y + 1 < _fluid._grid.get_size().y) {
				std::size_t ypos_index = _fluid_cell_indices(pos.x, pos.y + 1, pos.z);
				if (ypos_index != _not_a_fluid_cell) {
					neg_t += cell.fluid_ypos * z[ypos_index];
				}
			}
			if (pos.z + 1 < _fluid._grid.get_size().z) {
				std::size_t zpos_index = _fluid_cell_indices(pos.x, pos.y, pos.z + 1);
				if (zpos_index != _not_a_fluid_cell) {
					neg_t += cell.fluid_zpos * z[zpos_index];
				}
			}
			z[i] = (q_scratch[i] - _a_scale * precon[i] * neg_t) * precon[i];
		}
	}

	void pressure_solver::_apply_a(std::vector<double> &out, const std::vector<double> &v) const {
		for (std::size_t i = 0; i < _fluid_cells.size(); ++i) {
			vec3s pos = _fluid_cells[i];
			const cell_data &current = _a[i];
			double value = current.nonsolid_neighbors * v[i];

			if (pos.x > 0) {
				std::size_t xneg_id = _fluid_cell_indices(pos.x - 1, pos.y, pos.z);
				if (xneg_id != _not_a_fluid_cell) {
					value -= _a[xneg_id].fluid_xpos * v[xneg_id];
				}
			}
			if (pos.y > 0) {
				std::size_t yneg_id = _fluid_cell_indices(pos.x, pos.y - 1, pos.z);
				if (yneg_id != _not_a_fluid_cell) {
					value -= _a[yneg_id].fluid_ypos * v[yneg_id];
				}
			}
			if (pos.z > 0) {
				std::size_t zneg_id = _fluid_cell_indices(pos.x, pos.y, pos.z - 1);
				if (zneg_id != _not_a_fluid_cell) {
					value -= _a[zneg_id].fluid_zpos * v[zneg_id];
				}
			}

			if (pos.x + 1 < _fluid._grid.get_size().x) {
				std::size_t xpos_id = _fluid_cell_indices(pos.x + 1, pos.y, pos.z);
				if (xpos_id != _not_a_fluid_cell) {
					value -= current.fluid_xpos * v[xpos_id];
				}
			}
			if (pos.y + 1 < _fluid._grid.get_size().y) {
				std::size_t ypos_id = _fluid_cell_indices(pos.x, pos.y + 1, pos.z);
				if (ypos_id != _not_a_fluid_cell) {
					value -= current.fluid_ypos * v[ypos_id];
				}
			}
			if (pos.z + 1 < _fluid._grid.get_size().z) {
				std::size_t zpos_id = _fluid_cell_indices(pos.x, pos.y, pos.z + 1);
				if (zpos_id != _not_a_fluid_cell) {
					value -= current.fluid_zpos * v[zpos_id];
				}
			}

			out[i] = _a_scale * value;
		}
	}

	void pressure_solver::_muladd(
		std::vector<double> &out, const std::vector<double> &a, const std::vector<double> &b, double s
	) {
		for (std::size_t i = 0; i < a.size(); ++i) {
			out[i] = a[i] + s * b[i];
		}
	}
}
