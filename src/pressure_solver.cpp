#include "pressure_solver.h"

/// \file
/// Implementation of the pressure solver.

#include <algorithm>
#include <iostream>

namespace fluid {
	pressure_solver::cell_data::cell_data() : nonsolid_neighbors(0), fluid_xpos(0), fluid_ypos(0), fluid_zpos(0) {
	}


	pressure_solver::pressure_solver(simulation &sim, const std::vector<vec3s> &fluid_cells) :
		_fluid_cell_indices(sim.grid().grid().get_size(), _not_a_fluid_cell),
		_fluid_cells(fluid_cells), _sim(sim) {
	}

	std::tuple<std::vector<double>, double, std::size_t> pressure_solver::solve(double dt) {
		_compute_fluid_cell_indices();

		_a_scale = dt / (_sim.density * _sim.cell_size * _sim.cell_size);
		_compute_a_matrix();
		std::vector<double> b = _compute_b_vector();
		std::vector<double> precon = _compute_preconditioner(dt);
		double residual = 0.0;

		std::vector<double> p(_fluid_cells.size(), 0.0);
		double tot = 0.0;
		for (double bval : b) {
			tot += bval * bval;
		}
		if (tot < 1e-6) {
			return { p, 0.0, 0 };
		}
		std::vector<double> r = b;

		std::vector<double> z(_fluid_cells.size(), 0.0), q_scratch(_fluid_cells.size(), 0.0);
		_apply_preconditioner(z, q_scratch, precon, r);
		std::vector<double> s = z;

		double sigma = vec_ops::dynamic::dot(z, r);

		std::size_t i = 0;
		for (; i < max_iterations; ++i) {

			_apply_a(z, s);

			double alpha = sigma / vec_ops::dynamic::dot(z, s);

			_muladd(p, p, s, alpha);
			_muladd(r, r, z, -alpha);

			residual = *std::max_element(r.begin(), r.end());
			if (residual < tolerance) {
				++i;
				break;
			}

			_apply_preconditioner(z, q_scratch, precon, r);

			double sigma_new = vec_ops::dynamic::dot(z, r);

			double beta = sigma_new / sigma;

			_muladd(s, z, s, beta);

			sigma = sigma_new;
		}
		return { p, residual, i };
	}

	void pressure_solver::apply_pressure(double dt, const std::vector<double> &p) const {
		double _coeff = dt / (_sim.density * _sim.cell_size);

		for (std::size_t i = 0; i < _fluid_cells.size(); ++i) {
			vec3s pos = _fluid_cells[i];
			double cur_pressure = p[i];
			fluid_grid::cell &cell = _sim.grid().grid()(pos);

			{
				vec3s xpos = pos + vec3s::axis<0>();
				fluid_grid::cell::type type = _sim.grid().get_cell_and_type(xpos).second;
				if (type != fluid_grid::cell::type::solid) {
					double otherp = 0.0;
					if (type == fluid_grid::cell::type::fluid) {
						otherp = p[_fluid_cell_indices(xpos)];
					}
					cell.velocities_posface.x -= _coeff * (otherp - cur_pressure);
				} else {
					cell.velocities_posface.x = _usolid.x;
				}
			}

			{
				vec3s ypos = pos + vec3s::axis<1>();
				fluid_grid::cell::type type = _sim.grid().get_cell_and_type(ypos).second;
				if (type != fluid_grid::cell::type::solid) {
					double otherp = 0.0;
					if (type == fluid_grid::cell::type::fluid) {
						otherp = p[_fluid_cell_indices(ypos)];
					}
					cell.velocities_posface.y -= _coeff * (otherp - cur_pressure);
				} else {
					cell.velocities_posface.y = _usolid.y;
				}
			}

			{
				vec3s zpos = pos + vec3s::axis<2>();
				fluid_grid::cell::type type = _sim.grid().get_cell_and_type(zpos).second;
				if (type != fluid_grid::cell::type::solid) {
					double otherp = 0.0;
					if (type == fluid_grid::cell::type::fluid) {
						otherp = p[_fluid_cell_indices(zpos)];
					}
					cell.velocities_posface.z -= _coeff * (otherp - cur_pressure);
				} else {
					cell.velocities_posface.z = _usolid.z;
				}
			}

			// since non-fluid cells are not updated above, we update them here
			if (auto [xneg_cell, type] = _sim.grid().get_cell_and_type(pos - vec3s::axis<0>()); xneg_cell) {
				if (type == fluid_grid::cell::type::air) {
					xneg_cell->velocities_posface.x -= _coeff * cur_pressure;
				} else if (type == fluid_grid::cell::type::solid) {
					xneg_cell->velocities_posface.x = _usolid.x;
				}
			}

			if (auto [yneg_cell, type] = _sim.grid().get_cell_and_type(pos - vec3s::axis<1>()); yneg_cell) {
				if (type == fluid_grid::cell::type::air) {
					yneg_cell->velocities_posface.y -= _coeff * cur_pressure;
				} else if (type == fluid_grid::cell::type::solid) {
					yneg_cell->velocities_posface.y = _usolid.y;
				}
			}

			if (auto [zneg_cell, type] = _sim.grid().get_cell_and_type(pos - vec3s::axis<2>()); zneg_cell) {
				if (type == fluid_grid::cell::type::air) {
					zneg_cell->velocities_posface.z -= _coeff * cur_pressure;
				} else if (type == fluid_grid::cell::type::solid) {
					zneg_cell->velocities_posface.z = _usolid.z;
				}
			}
		}
	}

	void pressure_solver::_compute_fluid_cell_indices() {
		_fluid_cell_indices.fill(_not_a_fluid_cell);
		for (std::size_t i = 0; i < _fluid_cells.size(); ++i) {
			_fluid_cell_indices(_fluid_cells[i]) = i;
		}
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
					_sim.grid().get_cell_and_type(pos + o).second != fluid_grid::cell::type::solid ? 1 : 0;
			}
			data.fluid_xpos =
				_sim.grid().get_cell_and_type(pos + vec3s::axis<0>()).second == fluid_grid::cell::type::fluid ? 1 : 0;
			data.fluid_ypos =
				_sim.grid().get_cell_and_type(pos + vec3s::axis<1>()).second == fluid_grid::cell::type::fluid ? 1 : 0;
			data.fluid_zpos =
				_sim.grid().get_cell_and_type(pos + vec3s::axis<2>()).second == fluid_grid::cell::type::fluid ? 1 : 0;
			_a.emplace_back(data);
		}
	}

	std::vector<double> pressure_solver::_compute_b_vector() const {
		std::vector<double> b(_fluid_cells.size(), 0.0);
		double scale = 1.0 / _sim.cell_size;
		for (std::size_t i = 0; i < _fluid_cells.size(); ++i) {
			vec3s pos = _fluid_cells[i];
			vec3d vel = _sim.grid().grid()(pos).velocities_posface;

			double value = -(vel.x + vel.y + vel.z);

			if (pos.x > 0) {
				const fluid_grid::cell &cell = _sim.grid().grid()(pos.x - 1, pos.y, pos.z);
				value += cell.velocities_posface.x;
				if (cell.cell_type == fluid_grid::cell::type::solid) {
					value += _usolid.x - cell.velocities_posface.x;
				}
			} else {
				value += _usolid.x;
			}
			if (pos.y > 0) {
				const fluid_grid::cell &cell = _sim.grid().grid()(pos.x, pos.y - 1, pos.z);
				value += cell.velocities_posface.y;
				if (cell.cell_type == fluid_grid::cell::type::solid) {
					value += _usolid.y - cell.velocities_posface.y;
				}
			} else {
				value += _usolid.y;
			}
			if (pos.z > 0) {
				const fluid_grid::cell &cell = _sim.grid().grid()(pos.x, pos.y, pos.z - 1);
				value += cell.velocities_posface.z;
				if (cell.cell_type == fluid_grid::cell::type::solid) {
					value += _usolid.z - cell.velocities_posface.z;
				}
			} else {
				value += _usolid.z;
			}

			if (_sim.grid().get_cell_and_type(pos + vec3s::axis<0>()).second == fluid_grid::cell::type::solid) {
				value += vel.x - _usolid.x;
			}
			if (_sim.grid().get_cell_and_type(pos + vec3s::axis<1>()).second == fluid_grid::cell::type::solid) {
				value += vel.y - _usolid.y;
			}
			if (_sim.grid().get_cell_and_type(pos + vec3s::axis<2>()).second == fluid_grid::cell::type::solid) {
				value += vel.z - _usolid.z;
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

			if (std::size_t xneg_index = _get_neg_neighbor_index<0>(pos); xneg_index != _not_a_fluid_cell) {
				const cell_data &xneg_cell = _a[xneg_index];
				double
					xneg_precon = precon[xneg_index],
					a_times_precon = xneg_cell.fluid_xpos * xneg_precon;
				neg_e += a_times_precon * a_times_precon;
				neg_e_tau +=
					(xneg_cell.fluid_xpos * (xneg_cell.fluid_ypos + xneg_cell.fluid_zpos)) *
					xneg_precon * xneg_precon;
			}

			if (std::size_t yneg_index = _get_neg_neighbor_index<1>(pos); yneg_index != _not_a_fluid_cell) {
				const cell_data &yneg_cell = _a[yneg_index];
				double
					yneg_precon = precon[yneg_index],
					a_times_precon = yneg_cell.fluid_ypos * yneg_precon;
				neg_e += a_times_precon * a_times_precon;
				neg_e_tau +=
					(yneg_cell.fluid_ypos * (yneg_cell.fluid_xpos + yneg_cell.fluid_zpos)) *
					yneg_precon * yneg_precon;
			}

			if (std::size_t zneg_index = _get_neg_neighbor_index<2>(pos); zneg_index != _not_a_fluid_cell) {
				const cell_data &zneg_cell = _a[zneg_index];
				double
					zneg_precon = precon[zneg_index],
					a_times_precon = zneg_cell.fluid_zpos * zneg_precon;
				neg_e += a_times_precon * a_times_precon;
				neg_e_tau +=
					(zneg_cell.fluid_zpos * (zneg_cell.fluid_xpos + zneg_cell.fluid_ypos)) *
					zneg_precon * zneg_precon;
			}

			// still needs to be scaled by _a_scale later
			double e = _a[i].nonsolid_neighbors - (neg_e + tau * neg_e_tau) * _a_scale;

			if (e < sigma * _a[i].nonsolid_neighbors) {
				e = static_cast<double>(_a[i].nonsolid_neighbors);
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
			if (std::size_t xneg_index = _get_neg_neighbor_index<0>(pos); xneg_index != _not_a_fluid_cell) {
				neg_t += _a[xneg_index].fluid_xpos * precon[xneg_index] * q_scratch[xneg_index];
			}
			if (std::size_t yneg_index = _get_neg_neighbor_index<1>(pos); yneg_index != _not_a_fluid_cell) {
				neg_t += _a[yneg_index].fluid_ypos * precon[yneg_index] * q_scratch[yneg_index];
			}
			if (std::size_t zneg_index = _get_neg_neighbor_index<2>(pos); zneg_index != _not_a_fluid_cell) {
				neg_t += _a[zneg_index].fluid_zpos * precon[zneg_index] * q_scratch[zneg_index];
			}
			q_scratch[i] = (r[i] + _a_scale * neg_t) * precon[i];
		}
		// next solve L^T z = q
		for (std::size_t i = _fluid_cells.size(); i > 0; ) {
			--i;
			vec3s pos = _fluid_cells[i];
			const cell_data &cell = _a[i];
			double neg_t = 0.0; // the negative part of t that needs to be scaled by _a_scale * precon[i]
			if (std::size_t xpos_index = _get_pos_neighbor_index<0>(pos); xpos_index != _not_a_fluid_cell) {
				neg_t += cell.fluid_xpos * z[xpos_index];
			}
			if (std::size_t ypos_index = _get_pos_neighbor_index<1>(pos); ypos_index != _not_a_fluid_cell) {
				neg_t += cell.fluid_ypos * z[ypos_index];
			}
			if (std::size_t zpos_index = _get_pos_neighbor_index<2>(pos); zpos_index != _not_a_fluid_cell) {
				neg_t += cell.fluid_zpos * z[zpos_index];
			}
			z[i] = (q_scratch[i] + _a_scale * precon[i] * neg_t) * precon[i];
		}
	}

	void pressure_solver::_apply_a(std::vector<double> &out, const std::vector<double> &v) const {
		for (std::size_t i = 0; i < _fluid_cells.size(); ++i) {
			vec3s pos = _fluid_cells[i];
			const cell_data &current = _a[i];
			double value = current.nonsolid_neighbors * v[i];

			if (std::size_t xneg_id = _get_neg_neighbor_index<0>(pos); xneg_id != _not_a_fluid_cell) {
				value -= _a[xneg_id].fluid_xpos * v[xneg_id];
			}
			if (std::size_t yneg_id = _get_neg_neighbor_index<1>(pos); yneg_id != _not_a_fluid_cell) {
				value -= _a[yneg_id].fluid_ypos * v[yneg_id];
			}
			if (std::size_t zneg_id = _get_neg_neighbor_index<2>(pos); zneg_id != _not_a_fluid_cell) {
				value -= _a[zneg_id].fluid_zpos * v[zneg_id];
			}

			if (std::size_t xpos_id = _get_pos_neighbor_index<0>(pos); xpos_id != _not_a_fluid_cell) {
				value -= current.fluid_xpos * v[xpos_id];
			}
			if (std::size_t ypos_id = _get_pos_neighbor_index<1>(pos); ypos_id != _not_a_fluid_cell) {
				value -= current.fluid_ypos * v[ypos_id];
			}
			if (std::size_t zpos_id = _get_pos_neighbor_index<2>(pos); zpos_id != _not_a_fluid_cell) {
				value -= current.fluid_zpos * v[zpos_id];
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
