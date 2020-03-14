#include "simulation.h"

/// \file
/// Implementation of the fluid simulation.

#include <cmath>
#include <algorithm>
#include <iostream>
#include <random>

#include "pressure_solver.h"

namespace fluid {
	void simulation::resize(vec3s sz) {
		_grid = fluid_grid(sz);
		_space_hash.resize(sz);
	}

	void simulation::update(double dt) {
		std::cout << "update\n";
		while (true) {
			double ts = 3.0 * cfl();
			if (ts > dt) {
				time_step(dt);
				break;
			}
			time_step(ts);
			dt -= ts;
		}
	}

	void simulation::time_step(double dt) {
		std::cout << "timestep " << dt << "\n";
		_advect_particles(dt);
		_hash_particles();
		_transfer_to_grid();
		// add external forces
		for (std::size_t z = 0; z < grid().grid().get_size().z; ++z) {
			for (std::size_t y = 0; y < grid().grid().get_size().y; ++y) {
				for (std::size_t x = 0; x < grid().grid().get_size().x; ++x) {
					grid().grid()(x, y, z).velocities_posface += vec3d(0.0, -981.0, 0.0) * dt;
				}
			}
		}
		// solve and apply pressure
		{
			std::vector<vec3s> fluid_cells = _space_hash.get_sorted_occupied_cells();
			pressure_solver solver(*this, fluid_cells);
			auto [pressure, residual, iters] = solver.solve(dt);
			std::cerr << "  iterations = " << iters << "\n";
			std::cerr << "  residual = " << residual << "\n";
			std::cerr << "  max pressure = " << *std::max_element(pressure.begin(), pressure.end()) << "\n";
			assert(iters < solver.max_iterations);
			solver.apply_pressure(dt, pressure);
			double maxv = 0.0;
			for (const particle &p : _particles) {
				maxv = std::max(maxv, p.velocity.squared_length());
			}
			std::cerr << "  max velocity = " << std::sqrt(maxv) << "\n";
		}
		_transfer_from_grid();
	}

	void simulation::seed_box(vec3s start, vec3s size, std::size_t density) {
		std::minstd_rand rand(std::random_device{}());
		double small_cell_size = cell_size / density;
		std::uniform_real_distribution<double> dist(0.0, small_cell_size);
		vec3d base = grid_offset + vec3d(start) * cell_size;
		for (std::size_t z = 0; z < size.z; ++z) {
			for (std::size_t y = 0; y < size.y; ++y) {
				for (std::size_t x = 0; x < size.x; ++x) {
					vec3d cell_offset = vec3d(x, y, z) * cell_size;
					for (std::size_t sx = 0; sx < density; ++sx) {
						for (std::size_t sy = 0; sy < density; ++sy) {
							for (std::size_t sz = 0; sz < density; ++sz) {
								vec3d offset(dist(rand), dist(rand), dist(rand));
								particle p;
								p.position =
									offset + vec3d(sx, sy, sz) * small_cell_size + cell_offset + base;
								_particles.emplace_back(p);
							}
						}
					}
				}
			}
		}
	}

	double simulation::cfl() const {
		double maxlen = 0.0f;
		for (const particle &p : get_particles()) {
			maxlen = std::max(maxlen, p.velocity.squared_length());
		}
		return cell_size / std::sqrt(maxlen);
	}

	double simulation::_kernel(vec3d p) const {
		// TODO more kernels
		// this is the linear kernel
		p /= cell_size;
		return std::max((1.0 - std::abs(p.x)) * (1.0 - std::abs(p.y)) * (1.0 - std::abs(p.z)), 0.0);
	}

	void simulation::_advect_particles(double dt) {
		vec3d max_corner = cell_size * vec3d(grid().grid().get_size()) + grid_offset;
		for (particle &p : _particles) {
			p.position += p.velocity * dt;
			// clamp the particle back into the grid
			vec_ops::apply_to(p.position, std::clamp<double>, p.position, grid_offset, max_corner);
		}
	}

	void simulation::_hash_particles() {
		_space_hash.clear();
		for (particle &p : _particles) {
			vec3d grid_pos = (p.position - grid_offset) / cell_size;
			p.grid_index = vec_ops::apply<vec3s>(
				[](double pos, std::size_t max) {
					return std::min(static_cast<std::size_t>(std::max(pos, 0.0)), max - 1);
				},
				grid_pos, grid().grid().get_size()
					);
			_space_hash.add_object_at(p.grid_index, p);
		}
	}

	void simulation::_transfer_to_grid() {
		double half_cell = 0.5 * cell_size;
		double zpos = grid_offset.z + half_cell;
		for (std::size_t z = 0; z < grid().grid().get_size().z; ++z, zpos += cell_size) {
			double zpos_face = zpos + half_cell, ypos = grid_offset.y + half_cell;
			for (std::size_t y = 0; y < grid().grid().get_size().y; ++y, ypos += cell_size) {
				double ypos_face = ypos + half_cell, xpos = grid_offset.x + half_cell;
				for (std::size_t x = 0; x < grid().grid().get_size().x; ++x, xpos += cell_size) {
					double xpos_face = xpos + half_cell;

					vec3d sum_vel, sum_weight;
					vec3d
						xface(xpos_face, ypos, zpos),
						yface(xpos, ypos_face, zpos),
						zface(xpos, ypos, zpos_face);
					_space_hash.for_all_nearby_objects(
						vec3s(x, y, z), vec3s(1, 1, 1), vec3s(1, 1, 1),
						[&](particle &p) {
							vec3d weights(
								_kernel(p.position - xface),
								_kernel(p.position - yface),
								_kernel(p.position - zface)
							);
							sum_weight += weights;
							sum_vel += vec_ops::memberwise::mul(weights, p.velocity);
						}
					);
					fluid_grid::cell &cell = grid().grid()(x, y, z);
					cell.cell_type = fluid_grid::cell::type::air;
					if (!_space_hash.get_objects_at(vec3s(x, y, z)).empty()) {
						cell.cell_type = fluid_grid::cell::type::fluid;
					}
					vec_ops::apply_to(
						cell.velocities_posface,
						[](double vel, double weight) {
							return weight > 1e-6 ? vel / weight : 0.0; // TODO magic number
						},
						sum_vel, sum_weight
							);
				}
			}
		}
	}

	void simulation::_transfer_from_grid() {
		for (particle &p : _particles) {
			const fluid_grid::cell &cell = grid().grid()(p.grid_index);
			vec3d neg_vel;
			if (p.grid_index.x > 0) {
				neg_vel.x = grid().grid()(p.grid_index - vec3s::axis<0>()).velocities_posface.x;
			}
			if (p.grid_index.y > 0) {
				neg_vel.y = grid().grid()(p.grid_index - vec3s::axis<1>()).velocities_posface.y;
			}
			if (p.grid_index.z > 0) {
				neg_vel.z = grid().grid()(p.grid_index - vec3s::axis<2>()).velocities_posface.z;
			}
			vec3d t = (p.position - grid_offset) / cell_size - vec3d(p.grid_index);
			vec_ops::apply_to(
				p.velocity,
				[](double a, double b, double t) {
					return a * (1.0 - t) + b * t;
				},
				neg_vel, cell.velocities_posface, t
					);
		}
	}
}
