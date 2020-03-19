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
		std::cout << "  timestep " << dt << "\n";
		_advect_particles(dt);
		_hash_particles();
		_transfer_to_grid_pic_flip();
		_old_grid = _grid;
		_remove_boundary_velocities(_old_grid);
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
			if (iters >= solver.max_iterations) {
				std::cerr << "    WARNING: maximum iterations exceeded\n";
			}
			solver.apply_pressure(dt, pressure);

			std::cerr << "    iterations = " << iters << "\n";
			std::cerr << "    residual = " << residual << "\n";
			std::cerr << "    max pressure = " << *std::max_element(pressure.begin(), pressure.end()) << "\n";
			double maxv = 0.0;
			for (const particle &p : _particles) {
				maxv = std::max(maxv, p.velocity.squared_length());
			}
			std::cerr << "    max velocity = " << std::sqrt(maxv) << "\n";
		}
		_add_spring_forces(dt, 1, 0);
		/*_transfer_from_grid_pic();*/
		_transfer_from_grid_flip(1.0);
		/*_transfer_from_grid_flip(0.95);*/
	}

	void simulation::seed_box(vec3s start, vec3s size, std::size_t density) {
		double small_cell_size = cell_size / density;
		std::uniform_real_distribution<double> dist(0.0, small_cell_size);
		vec3d base = grid_offset + vec3d(start) * cell_size;
		for (std::size_t z = 0; z < size.z; ++z) {
			for (std::size_t y = 0; y < size.y; ++y) {
				for (std::size_t x = 0; x < size.x; ++x) {
					vec3d cell_offset = vec3d(vec3s(x, y, z)) * cell_size;
					for (std::size_t sx = 0; sx < density; ++sx) {
						for (std::size_t sy = 0; sy < density; ++sy) {
							for (std::size_t sz = 0; sz < density; ++sz) {
								vec3d offset(dist(random), dist(random), dist(random));
								particle p;
								p.position =
									offset + vec3d(vec3s(sx, sy, sz)) * small_cell_size + cell_offset + base;
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
		for (const particle &p : particles()) {
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
		vec3d
			skin_width = vec3d(boundary_skin_width, boundary_skin_width, boundary_skin_width),
			min_corner = grid_offset + skin_width,
			max_corner = cell_size * vec3d(grid().grid().get_size()) + grid_offset - skin_width;
		for (particle &p : _particles) {
			p.position += p.velocity * dt;
			// clamp the particle back into the grid
			vec_ops::apply_to(p.position, std::clamp<double>, p.position, min_corner, max_corner);
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

	void simulation::_transfer_to_grid_pic_flip() {
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

	vec3d simulation::_get_negative_face_velocities(const fluid_grid &grid, vec3s id) {
		vec3d neg_vel;
		if (id.x > 0) {
			neg_vel.x = grid.grid()(id - vec3s::axis<0>()).velocities_posface.x;
		}
		if (id.y > 0) {
			neg_vel.y = grid.grid()(id - vec3s::axis<1>()).velocities_posface.y;
		}
		if (id.z > 0) {
			neg_vel.z = grid.grid()(id - vec3s::axis<2>()).velocities_posface.z;
		}
		return neg_vel;
	}

	void simulation::_remove_boundary_velocities(fluid_grid &g) {
		vec3s max_pos = g.grid().get_size() - vec3s(1, 1, 1);
		for (std::size_t z = 0; z < g.grid().get_size().z; ++z) {
			for (std::size_t y = 0; y < g.grid().get_size().y; ++y) {
				g.grid()(max_pos.x, y, z).velocities_posface.x = 0.0;
			}
			for (std::size_t x = 0; x < g.grid().get_size().x; ++x) {
				g.grid()(x, max_pos.y, z).velocities_posface.y = 0.0;
			}
		}
		for (std::size_t y = 0; y < g.grid().get_size().y; ++y) {
			for (std::size_t x = 0; x < g.grid().get_size().x; ++x) {
				g.grid()(x, y, max_pos.z).velocities_posface.z = 0.0;
			}
		}
	}

	void simulation::_transfer_from_grid_pic() {
		for (particle &p : _particles) {
			const fluid_grid::cell &cell = grid().grid()(p.grid_index);
			vec3d t = (p.position - grid_offset) / cell_size - vec3d(p.grid_index);
			vec_ops::apply_to(
				p.velocity,
				[](double a, double b, double t) {
					return a * (1.0 - t) + b * t;
				},
				_get_negative_face_velocities(_grid, p.grid_index), cell.velocities_posface, t
					);
		}
	}

	void simulation::_transfer_from_grid_flip(double blend) {
		for (particle &p : _particles) {
			vec3d t = (p.position - grid_offset) / cell_size - vec3d(p.grid_index);
			vec3d
				old_velocity = vec_ops::apply<vec3d>(
					lerp<double>,
					_get_negative_face_velocities(_old_grid, p.grid_index),
					_old_grid.grid()(p.grid_index).velocities_posface,
					t
					),
				new_velocity = vec_ops::apply<vec3d>(
					lerp<double>,
					_get_negative_face_velocities(_grid, p.grid_index),
					_grid.grid()(p.grid_index).velocities_posface,
					t
					);
			p.velocity = new_velocity + (p.velocity - old_velocity) * blend;
		}
	}

	void simulation::_add_spring_forces(double dt, std::size_t step, std::size_t substep) {
		double re = cell_size / std::sqrt(2.0); // some kind of radius
		// in apic2d, this distribution is actually (0, 1), not sure why
		std::uniform_real_distribution<double> dist(-1.0, 1.0);
		double min_dist = 0.1 * re;
		std::vector<vec3d> new_positions(_particles.size());
#pragma omp parallel
		for (std::size_t i = substep; i < _particles.size(); i += step) {
			const particle &p = _particles[i];
			vec3d spring;
			_space_hash.for_all_nearby_objects(
				p.grid_index, vec3s(1, 1, 1), vec3s(1, 1, 1),
				[&](const particle &other) {
					if (&other != &p) {
						vec3d offset = p.position - other.position;
						double sqr_dist = offset.squared_length();
						if (sqr_dist < min_dist * min_dist) {
							// the two particles are not too far away, so just add a random force to avoid
							// floating point errors
							// apic2d multiplies this value by 0.01 * dt here
							spring += re * vec3d(dist(random), dist(random), dist(random));
						} else {
							double kernel_lower = 1.0 - sqr_dist / (cell_size * cell_size);
							double kernel = 0.0;
							if (kernel_lower > 0.0) {
								kernel = kernel_lower * kernel_lower * kernel_lower;
							}
							spring += (kernel * re / std::sqrt(sqr_dist)) * offset;
						}
					}
				}
			);
			new_positions[i] = p.position + spring * dt;
		}
		for (std::size_t i = substep; i < _particles.size(); i += step) {
			_particles[i].position = new_positions[i];
		}
	}
}
