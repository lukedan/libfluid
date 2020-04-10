#include "fluid/simulation.h"

/// \file
/// Implementation of the fluid simulation.

#include <cmath>
#include <algorithm>
#include <random>

#include "fluid/pressure_solver.h"

namespace fluid {
	void simulation::resize(vec3s sz) {
		_grid = fluid_grid(sz);
		_space_hash.resize(sz);
	}

	void simulation::update(double dt) {
		while (true) {
			double ts = cfl_number * cfl();
			if (ts > dt) {
				time_step(dt);
				break;
			}
			time_step(ts);
			dt -= ts;
		}
	}

	void simulation::time_step(double dt) {
		if (pre_time_step_callback) {
			pre_time_step_callback(dt);
		}

		_advect_particles(dt);
		if (post_advection_callback) {
			post_advection_callback(dt);
		}

		hash_particles();
		_update_sources();

		_transfer_to_grid();
		if (post_particle_to_grid_transfer_callback) {
			post_particle_to_grid_transfer_callback(dt);
		}

		// add gravity
		for (std::size_t z = 0; z < grid().grid().get_size().z; ++z) {
			for (std::size_t y = 0; y < grid().grid().get_size().y; ++y) {
				for (std::size_t x = 0; x < grid().grid().get_size().x; ++x) {
					grid().grid()(x, y, z).velocities_posface += gravity * dt;
				}
			}
		}
		if (post_gravity_callback) {
			post_gravity_callback(dt);
		}

		// solve and apply pressure
		{
			std::vector<vec3s> fluid_cells = _space_hash.get_sorted_occupied_cells();
			// remove solid cells
			auto end = fluid_cells.end();
			for (auto it = fluid_cells.begin(); it != end; ) {
				if (grid().grid()(*it).cell_type == fluid_grid::cell::type::solid) {
					std::swap(*--end, *it);
				} else {
					++it;
				}
			}
			fluid_cells.erase(end, fluid_cells.end());

			pressure_solver solver(*this, fluid_cells);
			auto [pressure, residual, iters] = solver.solve(dt);
			if (post_pressure_solve_callback) {
				post_pressure_solve_callback(dt, pressure, residual, iters);
			}

			solver.apply_pressure(dt, pressure);
			if (post_apply_pressure_callback) {
				post_apply_pressure_callback(dt);
			}
		}

		_correct_positions(dt);
		if (post_correction_callback) {
			post_correction_callback(dt);
		}

		_transfer_from_grid();
		if (post_grid_to_particle_transfer_callback) {
			post_grid_to_particle_transfer_callback(dt);
		}
	}

	void simulation::time_step() {
		time_step(std::min(cfl_number * cfl(), 0.033));
	}

	void simulation::reset_space_hash() {
		_space_hash.clear();
	}

	void simulation::seed_cell(vec3s cell, vec3d velocity, std::size_t density) {
		std::size_t
			index = grid().grid().index_to_raw(cell),
			num = _space_hash.get_num_objects_at(cell),
			target = density * density * density;
		std::uniform_real_distribution<double> dist(0.0, cell_size);
		vec3d offset = grid_offset + vec3d(cell) * cell_size;
		for (; num < target; ++num) {
			particle p;
			p.grid_index = cell;
			p.position = offset + vec3d(dist(random), dist(random), dist(random));
			p.velocity = velocity;
			_space_hash.add_object_at_raw(index, &p);
			_particles.emplace_back(p);
		}
	}

	void simulation::seed_box(vec3d start, vec3d size, std::size_t density) {
		vec3d end = start + size;
		vec3s
			start_cell = world_position_to_cell_index_unclamped(start),
			end_cell = world_position_to_cell_index_unclamped(end);
		seed_func(
			start_cell, end_cell - start_cell + vec3s(1, 1, 1),
			[&](vec3d pos) {
				return
					pos.x > start.x && pos.y > start.y && pos.z > start.z &&
					pos.x < end.x && pos.y < end.y && pos.z < end.z;
			},
			density
				);
	}

	void simulation::seed_sphere(vec3d center, double radius, std::size_t density) {
		vec3s
			start_cell = world_position_to_cell_index_unclamped(center - vec3d(radius, radius, radius)),
			end_cell = world_position_to_cell_index_unclamped(center + vec3d(radius, radius, radius));
		double sqr_radius = radius * radius;
		seed_func(
			start_cell, end_cell - start_cell + vec3s(1, 1, 1),
			[&](vec3d pos) {
				return (pos - center).squared_length() < sqr_radius;
			},
			density
				);
	}

	vec3s simulation::world_position_to_cell_index(vec3d pos) const {
		return vec_ops::apply<vec3s>(
			static_cast<const std::size_t & (*)(const std::size_t&, const std::size_t&)>(std::min),
			world_position_to_cell_index_unclamped(pos), grid().grid().get_size()
		);
	}

	vec3s simulation::world_position_to_cell_index_unclamped(vec3d pos) const {
		return vec_ops::apply<vec3s>(
			[](double v) {
				return static_cast<std::size_t>(std::max(v, 0.0));
			},
			(pos - grid_offset) / cell_size
				);
	}

	double simulation::cfl() const {
		double maxlen = 0.0;
		for (const particle &p : particles()) {
			maxlen = std::max(maxlen, p.velocity.squared_length());
		}
		return cell_size / std::sqrt(maxlen);
	}

	double simulation::_kernel(vec3d p) const {
		// linear kernel
		return
			std::max(0.0, 1.0 - std::abs(p.x)) *
			std::max(0.0, 1.0 - std::abs(p.y)) *
			std::max(0.0, 1.0 - std::abs(p.z));
	}

	vec3d simulation::_grad_kernel(vec3d p) const {
		vec3d neg_sign = vec_ops::apply<vec3d>(
			[](double d) {
				return d > 0.0 ? -1.0 : 1.0;
			},
			p
				);
		vec3d n = vec3d(1.0, 1.0, 1.0) - vec_ops::apply<vec3d>(static_cast<double (*)(double)>(std::abs), p);
		return vec3d(neg_sign.x * n.y * n.z, n.x * neg_sign.y * n.z, n.x * n.y * neg_sign.z) / cell_size;
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

	void simulation::hash_particles() {
		_space_hash.clear();
		for (particle &p : _particles) {
			vec3d grid_pos = (p.position - grid_offset) / cell_size;
			p.grid_index = vec_ops::apply<vec3s>(
				[](double pos, std::size_t max) {
					return std::min(static_cast<std::size_t>(std::max(pos, 0.0)), max - 1);
				},
				grid_pos, grid().grid().get_size()
					);
			_space_hash.add_object_at_unchecked(p.grid_index, &p);
		}
	}

	void simulation::_transfer_to_grid_pic() {
		double half_cell = 0.5 * cell_size;
		double zpos = grid_offset.z + half_cell;
		for (std::size_t z = 0; z < grid().grid().get_size().z; ++z, zpos += cell_size) {
			double zpos_face = zpos + half_cell, ypos = grid_offset.y + half_cell;
			for (std::size_t y = 0; y < grid().grid().get_size().y; ++y, ypos += cell_size) {
				double ypos_face = ypos + half_cell, xpos = grid_offset.x + half_cell;
				for (std::size_t x = 0; x < grid().grid().get_size().x; ++x, xpos += cell_size) {
					fluid_grid::cell &cell = grid().grid()(x, y, z);
					if (cell.cell_type != fluid_grid::cell::type::solid) {
						double xpos_face = xpos + half_cell;

						vec3d sum_vel, sum_weight;
						vec3d
							xface(xpos_face, ypos, zpos),
							yface(xpos, ypos_face, zpos),
							zface(xpos, ypos, zpos_face);
						_space_hash.for_all_nearby_objects(
							vec3s(x, y, z), vec3s(1, 1, 1), vec3s(1, 1, 1),
							[&](particle *p) {
								vec3d weights(
									_kernel((p->position - xface) / cell_size),
									_kernel((p->position - yface) / cell_size),
									_kernel((p->position - zface) / cell_size)
								);
								sum_weight += weights;
								sum_vel += vec_ops::memberwise::mul(weights, p->velocity);
							}
						);

						cell.cell_type = fluid_grid::cell::type::air;
						if (_space_hash.get_num_objects_at(vec3s(x, y, z)) != 0) {
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
	}

	void simulation::_transfer_to_grid_flip() {
		_transfer_to_grid_pic();
		_old_grid = _grid;
		_remove_boundary_velocities(_old_grid);
	}

	void simulation::_transfer_to_grid_apic() {
		double half_cell = 0.5 * cell_size;
		double zpos = grid_offset.z + half_cell;
		for (std::size_t z = 0; z < grid().grid().get_size().z; ++z, zpos += cell_size) {
			double zpos_face = zpos + half_cell, ypos = grid_offset.y + half_cell;
			for (std::size_t y = 0; y < grid().grid().get_size().y; ++y, ypos += cell_size) {
				double ypos_face = ypos + half_cell, xpos = grid_offset.x + half_cell;
				for (std::size_t x = 0; x < grid().grid().get_size().x; ++x, xpos += cell_size) {
					fluid_grid::cell &cell = grid().grid()(x, y, z);
					if (cell.cell_type != fluid_grid::cell::type::solid) {
						double xpos_face = xpos + half_cell;

						vec3d sum_vel, sum_weight;
						vec3d
							xface(xpos_face, ypos, zpos),
							yface(xpos, ypos_face, zpos),
							zface(xpos, ypos, zpos_face);
						_space_hash.for_all_nearby_objects(
							vec3s(x, y, z), vec3s(1, 1, 1), vec3s(1, 1, 1),
							[&](particle *p) {
								vec3d
									weights(
										_kernel(p->position - xface),
										_kernel(p->position - yface),
										_kernel(p->position - zface)
									),
									affine(
										vec_ops::dot(p->cx, xface - p->position),
										vec_ops::dot(p->cy, yface - p->position),
										vec_ops::dot(p->cz, zface - p->position)
									);
								sum_weight += weights;
								sum_vel += vec_ops::memberwise::mul(weights, p->velocity + affine);
							}
						);

						cell.cell_type = fluid_grid::cell::type::air;
						if (_space_hash.get_num_objects_at(vec3s(x, y, z)) != 0) {
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
		_remove_boundary_velocities(_grid);
	}

	void simulation::_transfer_to_grid() {
		switch (simulation_method) {
		case method::pic:
			_transfer_to_grid_pic();
			break;
		case method::flip_blend:
			_transfer_to_grid_flip();
			break;
		case method::apic:
			_transfer_to_grid_apic();
			break;
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
		if (g.grid().get_array_size(g.grid().get_size()) > 0) {
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
	}

	void simulation::_transfer_from_grid_pic() {
		for (particle &p : _particles) {
			vec3d t = (p.position - grid_offset) / cell_size - vec3d(p.grid_index);
			auto [v, tmid] = grid().get_face_samples(p.grid_index, t);
			p.velocity.x = trilerp(
				v.v000.x, v.v001.x, v.v010.x, v.v011.x, v.v100.x, v.v101.x, v.v110.x, v.v111.x, tmid.z, tmid.y, t.x
			);
			p.velocity.y = trilerp(
				v.v000.y, v.v001.y, v.v010.y, v.v011.y, v.v100.y, v.v101.y, v.v110.y, v.v111.y, tmid.z, t.y, tmid.x
			);
			p.velocity.z = trilerp(
				v.v000.z, v.v001.z, v.v010.z, v.v011.z, v.v100.z, v.v101.z, v.v110.z, v.v111.z, t.z, tmid.y, tmid.x
			);
		}
	}

	void simulation::_transfer_from_grid_flip(double blend) {
		for (particle &p : _particles) {
			vec3d t = (p.position - grid_offset) / cell_size - vec3d(p.grid_index);
			auto [v_old, tmid] = _old_grid.get_face_samples(p.grid_index, t);
			auto [v_new, tmid_other] = grid().get_face_samples(p.grid_index, t);
			vec3d
				old_velocity(
					trilerp(
						v_old.v000.x, v_old.v001.x, v_old.v010.x, v_old.v011.x,
						v_old.v100.x, v_old.v101.x, v_old.v110.x, v_old.v111.x,
						tmid.z, tmid.y, t.x
					),
					trilerp(
						v_old.v000.y, v_old.v001.y, v_old.v010.y, v_old.v011.y,
						v_old.v100.y, v_old.v101.y, v_old.v110.y, v_old.v111.y,
						tmid.z, t.y, tmid.x
					),
					trilerp(
						v_old.v000.z, v_old.v001.z, v_old.v010.z, v_old.v011.z,
						v_old.v100.z, v_old.v101.z, v_old.v110.z, v_old.v111.z,
						t.z, tmid.y, tmid.x
					)
				),
				new_velocity(
					trilerp(
						v_new.v000.x, v_new.v001.x, v_new.v010.x, v_new.v011.x,
						v_new.v100.x, v_new.v101.x, v_new.v110.x, v_new.v111.x,
						tmid.z, tmid.y, t.x
					),
					trilerp(
						v_new.v000.y, v_new.v001.y, v_new.v010.y, v_new.v011.y,
						v_new.v100.y, v_new.v101.y, v_new.v110.y, v_new.v111.y,
						tmid.z, t.y, tmid.x
					),
					trilerp(
						v_new.v000.z, v_new.v001.z, v_new.v010.z, v_new.v011.z,
						v_new.v100.z, v_new.v101.z, v_new.v110.z, v_new.v111.z,
						t.z, tmid.y, tmid.x
					)
				);
			p.velocity = new_velocity + (p.velocity - old_velocity) * blend;
		}
	}

	vec3d simulation::_calculate_c_vector(
		double v000, double v001, double v010, double v011,
		double v100, double v101, double v110, double v111,
		double tx, double ty, double tz
	) const {
		return
			_grad_kernel(vec3d(tx, ty, tz)) * v000 +
			_grad_kernel(vec3d(tx - 1.0, ty, tz)) * v001 +
			_grad_kernel(vec3d(tx, ty - 1.0, tz)) * v010 +
			_grad_kernel(vec3d(tx - 1.0, ty - 1.0, tz)) * v011 +
			_grad_kernel(vec3d(tx, ty, tz - 1.0)) * v100 +
			_grad_kernel(vec3d(tx - 1.0, ty, tz - 1.0)) * v101 +
			_grad_kernel(vec3d(tx, ty - 1.0, tz - 1.0)) * v110 +
			_grad_kernel(vec3d(tx - 1.0, ty - 1.0, tz - 1.0)) * v111;
	}

	void simulation::_transfer_from_grid_apic() {
		for (particle &p : _particles) {
			vec3d t = (p.position - grid_offset) / cell_size - vec3d(p.grid_index);
			auto [v, tmid] = grid().get_face_samples(p.grid_index, t);
			p.velocity.x = trilerp(
				v.v000.x, v.v001.x, v.v010.x, v.v011.x, v.v100.x, v.v101.x, v.v110.x, v.v111.x, tmid.z, tmid.y, t.x
			);
			p.velocity.y = trilerp(
				v.v000.y, v.v001.y, v.v010.y, v.v011.y, v.v100.y, v.v101.y, v.v110.y, v.v111.y, tmid.z, t.y, tmid.x
			);
			p.velocity.z = trilerp(
				v.v000.z, v.v001.z, v.v010.z, v.v011.z, v.v100.z, v.v101.z, v.v110.z, v.v111.z, t.z, tmid.y, tmid.x
			);
			p.cx = _calculate_c_vector(
				v.v000.x, v.v001.x, v.v010.x, v.v011.x, v.v100.x, v.v101.x, v.v110.x, v.v111.x, t.x, tmid.y, tmid.z
			);
			p.cy = _calculate_c_vector(
				v.v000.y, v.v001.y, v.v010.y, v.v011.y, v.v100.y, v.v101.y, v.v110.y, v.v111.y, tmid.x, t.y, tmid.z
			);
			p.cz = _calculate_c_vector(
				v.v000.z, v.v001.z, v.v010.z, v.v011.z, v.v100.z, v.v101.z, v.v110.z, v.v111.z, tmid.x, tmid.y, t.z
			);
		}
	}

	void simulation::_transfer_from_grid() {
		switch (simulation_method) {
		case method::pic:
			_transfer_from_grid_pic();
			break;
		case method::flip_blend:
			_transfer_from_grid_flip(blending_factor);
			break;
		case method::apic:
			_transfer_from_grid_apic();
			break;
		}
	}

	void simulation::_correct_positions(double dt) {
		// "Preserving Fluid Sheets with Adaptively Sampled Anisotropic Particles"
		// https://github.com/ryichando/shiokaze/blob/53997a4dcaee9ae8c55dcdbd9077f95f1f6c052a/src/flip/macnbflip3.cpp#L377

		double re = cell_size / std::sqrt(2.0); // particle radius
		std::uniform_real_distribution<double> dist(-1.0, 1.0);
		std::vector<vec3d> new_positions(_particles.size());

		int isize = static_cast<int>(_particles.size());
#pragma omp parallel
		{
			pcg32 thread_rand(std::random_device{}());
#pragma omp for
			for (int i = 0; i < isize; ++i) {
				const particle &p = _particles[i];
				vec3d spring;
				_space_hash.for_all_nearby_objects(
					p.grid_index, vec3s(1, 1, 1), vec3s(1, 1, 1),
					[&](const particle *other) {
						if (other != &p) {
							vec3d offset = p.position - other->position;
							double sqr_dist = offset.squared_length();
							if (sqr_dist < 1e-12) {
								// the two particles are not too far away, weight is 1, so just add a random force
								// to avoid floating point errors
								spring += vec3d(dist(thread_rand), dist(thread_rand), dist(thread_rand));
							} else {
								double kernel_lower = 1.0 - sqr_dist / (re * re);
								double kernel = 0.0;
								if (kernel_lower > 0.0) {
									kernel = kernel_lower * kernel_lower * kernel_lower;
								}
								spring += (kernel / std::sqrt(sqr_dist)) * offset;
							}
						}
					}
				);
				new_positions[i] = p.position + spring * (dt * correction_stiffness * re);
			}
		}

		// apply new positions & clamp back to the grid
		vec3d grid_max = grid_offset + vec3d(grid().grid().get_size()) * cell_size;
		for (std::size_t i = 0; i < _particles.size(); ++i) {
			_particles[i].position = vec_ops::apply<vec3d>(
				std::clamp<double>, new_positions[i], grid_offset, grid_max
			);
			_particles[i].grid_index = vec3s((_particles[i].position - grid_offset) / cell_size);
		}
	}

	void simulation::_update_sources() {
		for (auto &src : sources) {
			if (!src->active) {
				continue;
			}
			for (vec3s v : src->cells) {
				if (src->coerce_velocity) {
					_space_hash.for_all_objects_in(
						v,
						[&src](particle *p) {
							p->velocity = src->velocity;
						}
					);
				}
				seed_cell(v, src->velocity, src->target_density_cubic_root);
			}
		}
	}
}
