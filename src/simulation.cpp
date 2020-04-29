#include "fluid/simulation.h"

/// \file
/// Implementation of the fluid simulation.

#include <cmath>
#include <algorithm>
#include <random>

#include "fluid/pressure_solver.h"

namespace fluid {
	vec3s simulation::particle::compute_cell_index(vec3d grid_offset, double cell_size) const {
		return vec3s((position - grid_offset) / cell_size);
	}

	std::pair<vec3s, vec3d> simulation::particle::compute_cell_index_and_position(
		vec3d grid_offset, double cell_size
	) const {
		vec3d float_index = (position - grid_offset) / cell_size;
		vec3s cell_index(float_index);
		return { cell_index, float_index - vec3d(cell_index) };
	}


	void simulation::resize(vec3s sz) {
		_grid = mac_grid(sz);
		_space_hash = grid3<_cell_particles>(sz);
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

		// store old positions for collision detection
		update_and_hash_particles();
		_advect_particles(dt);
		if (post_advection_callback) {
			post_advection_callback(dt);
		}

		if constexpr (precise_collision_detection) {
			_detect_collisions();
			for (particle &p : _particles) {
				p.old_position = p.position;
			}
		}

		update_and_hash_particles();
		_update_sources();
		hash_particles();

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

		std::vector<vec3s> fluid_cells;
		if constexpr (precise_collision_detection) {
			for (std::size_t raw : _fluid_cells) {
				fluid_cells.emplace_back(grid().grid().index_from_raw(raw));
			}
		} else {
			for (std::size_t raw : _fluid_cells) {
				if (grid().grid()[raw].cell_type != mac_grid::cell::type::solid) {
					fluid_cells.emplace_back(grid().grid().index_from_raw(raw));
				}
			}
		}

		// solve and apply pressure
		{
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
		_detect_collisions();
		for (particle &p : _particles) {
			p.old_position = p.position;
		}

		_extrapolate_velocities(fluid_cells);

		_transfer_from_grid();
		if (post_grid_to_particle_transfer_callback) {
			post_grid_to_particle_transfer_callback(dt);
		}
	}

	void simulation::time_step() {
		time_step(std::min(cfl_number * cfl(), 0.033));
	}

	void simulation::reset_space_hash() {
		_space_hash.fill(_cell_particles());
		_fluid_cells.clear();
	}

	void simulation::seed_cell(vec3s cell, vec3d velocity, std::size_t density) {
		std::size_t
			index = grid().grid().index_to_raw(cell),
			num = _space_hash(cell).count,
			target = density * density * density;
		std::uniform_real_distribution<double> dist(0.0, cell_size);
		vec3d offset = grid_offset + vec3d(cell) * cell_size;
		for (; num < target; ++num) {
			particle p;
			p.old_position = p.position = offset + vec3d(dist(random), dist(random), dist(random));
			p.velocity = velocity;
			p.raw_cell_index = index;
			_particles.emplace_back(p);
		}
		_space_hash(cell).count = target;
	}

	void simulation::seed_box(vec3d start, vec3d size, vec3d vel, std::size_t density) {
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
			vel, density
				);
	}

	void simulation::seed_sphere(vec3d center, double radius, vec3d vel, std::size_t density) {
		vec3s
			start_cell = world_position_to_cell_index_unclamped(center - vec3d(radius, radius, radius)),
			end_cell = world_position_to_cell_index_unclamped(center + vec3d(radius, radius, radius));
		double sqr_radius = radius * radius;
		seed_func(
			start_cell, end_cell - start_cell + vec3s(1, 1, 1),
			[&](vec3d pos) {
				return (pos - center).squared_length() < sqr_radius;
			},
			vel, density
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
		for (auto &src : sources) {
			if (src->active && src->coerce_velocity) {
				for (vec3s v : src->cells) {
					_cell_particles cell = _space_hash(v);
					for (std::size_t c = cell.begin, i = 0; i < cell.count; ++i, ++c) {
						particle &p = _particles[c];
						p.velocity = src->velocity;
						p.cx = p.cy = p.cz = vec3d();
					}
				}
			}
		}

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

	void simulation::update_and_hash_particles() {
		for (particle &p : _particles) {
			vec3d grid_pos = (p.position - grid_offset) / cell_size;
			vec3s grid_index = vec_ops::apply<vec3s>(
				[](double pos, std::size_t max) {
					return std::min(static_cast<std::size_t>(std::max(pos, 0.0)), max - 1);
				},
				grid_pos, grid().grid().get_size()
					);
			p.raw_cell_index = grid().grid().index_to_raw(grid_index);
		}

		hash_particles();
	}

	void simulation::hash_particles() {
		reset_space_hash();

		std::sort(_particles.begin(), _particles.end(), [](const particle &lhs, const particle &rhs) {
			return lhs.raw_cell_index < rhs.raw_cell_index;
		});

		if (_particles.size() > 0) {
			std::size_t
				last_cell = _particles.front().raw_cell_index, // the cell the last particle is in
				count = 1;
			_fluid_cells.emplace_back(last_cell);
			for (std::size_t i = 1; i < _particles.size(); ++i, ++count) {
				std::size_t cur_cell = _particles[i].raw_cell_index;
				if (cur_cell != last_cell) {
					_space_hash[last_cell].count = count;
					count = 0;

					_fluid_cells.emplace_back(cur_cell);
					_space_hash[cur_cell].begin = i;
					last_cell = cur_cell;
				}
			}
			_space_hash[last_cell].count = count;
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
					mac_grid::cell &cell = grid().grid()(x, y, z);
					double xpos_face = xpos + half_cell;

					vec3d sum_vel, sum_weight;
					vec3d
						xface(xpos_face, ypos, zpos),
						yface(xpos, ypos_face, zpos),
						zface(xpos, ypos, zpos_face);
					_for_all_nearby_particles(
						vec3s(x, y, z), vec3s(1, 1, 1), vec3s(1, 1, 1),
						[&](particle &p) {
							vec3d weights(
								_kernel((p.position - xface) / cell_size),
								_kernel((p.position - yface) / cell_size),
								_kernel((p.position - zface) / cell_size)
							);
							sum_weight += weights;
							sum_vel += vec_ops::memberwise::mul(weights, p.velocity);
						}
					);
					vec_ops::apply_to(
						cell.velocities_posface,
						[](double vel, double weight) {
							return weight > 1e-6 ? vel / weight : 0.0; // TODO magic number
						},
						sum_vel, sum_weight
							);

					if (cell.cell_type != mac_grid::cell::type::solid) {
						cell.cell_type = mac_grid::cell::type::air;
						if (_space_hash(x, y, z).count > 0) {
							cell.cell_type = mac_grid::cell::type::fluid;
						}
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
					mac_grid::cell &cell = grid().grid()(x, y, z);
					double xpos_face = xpos + half_cell;

					vec3d sum_vel, sum_weight;
					vec3d
						xface(xpos_face, ypos, zpos),
						yface(xpos, ypos_face, zpos),
						zface(xpos, ypos, zpos_face);
					_for_all_nearby_particles(
						vec3s(x, y, z), vec3s(1, 1, 1), vec3s(1, 1, 1),
						[&](particle &p) {
							vec3d
								weights(
									_kernel(p.position - xface),
									_kernel(p.position - yface),
									_kernel(p.position - zface)
								),
								affine(
									vec_ops::dot(p.cx, xface - p.position),
									vec_ops::dot(p.cy, yface - p.position),
									vec_ops::dot(p.cz, zface - p.position)
								);
							sum_weight += weights;
							sum_vel += vec_ops::memberwise::mul(weights, p.velocity + affine);
						}
					);
					vec_ops::apply_to(
						cell.velocities_posface,
						[](double vel, double weight) {
							return weight > 1e-6 ? vel / weight : 0.0; // TODO magic number
						},
						sum_vel, sum_weight
							);

					if (cell.cell_type != mac_grid::cell::type::solid) {
						cell.cell_type = mac_grid::cell::type::air;
						if (_space_hash(x, y, z).count > 0) {
							cell.cell_type = mac_grid::cell::type::fluid;
						}
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

	vec3d simulation::_get_negative_face_velocities(const mac_grid &grid, vec3s id) {
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

	void simulation::_remove_boundary_velocities(mac_grid &g) {
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
			auto [grid_index, t] = p.compute_cell_index_and_position(grid_offset, cell_size);
			auto [v, tmid] = grid().get_face_samples(grid_index, t);
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
			auto [grid_index, t] = p.compute_cell_index_and_position(grid_offset, cell_size);
			auto [v_old, tmid] = _old_grid.get_face_samples(grid_index, t);
			auto [v_new, tmid_other] = grid().get_face_samples(grid_index, t);
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
			auto [grid_index, t] = p.compute_cell_index_and_position(grid_offset, cell_size);
			auto [v, tmid] = grid().get_face_samples(grid_index, t);
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
				_for_all_nearby_particles(
					p.compute_cell_index(grid_offset, cell_size), vec3s(1, 1, 1), vec3s(1, 1, 1),
					[&](const particle &other) {
						if (&other != &p) {
							vec3d offset = p.position - other.position;
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
		}
	}

	void simulation::_detect_collisions() {
		int nparticles = static_cast<int>(_particles.size());
#pragma omp parallel for
		for (int i = 0; i < nparticles; ++i) {
			particle &p = _particles[static_cast<std::size_t>(i)];
			vec3d from = p.old_position, to = p.position;
			for (std::size_t i = 0; i < 3; ++i) {
				bool into_wall = false;
				grid().grid().march_cells(
					[this, &p, &from, &to, &into_wall](vec3i pos, std::size_t dim, vec3d normal, double t) {
						if (pos.x >= 0 && pos.y >= 0 && pos.z >= 0) {
							vec3s upos(pos);
							if (
								upos.x < grid().grid().get_size().x &&
								upos.y < grid().grid().get_size().y &&
								upos.z < grid().grid().get_size().z
							) {
								if (grid().grid()(pos.x, pos.y, pos.z).cell_type != mac_grid::cell::type::solid) {
									return true;
								}
							}
						}
						// collision
						p.velocity[dim] = 0.0;

						vec3d offset = to - from;
						t += boundary_skin_width / vec_ops::dot(offset, normal);
						t = std::max(t, 0.0);
						from = t * to + (1.0 - t) * from;
						to[dim] = from[dim];

						into_wall = true;
						return false;
					},
					(from - grid_offset) / cell_size, (to - grid_offset) / cell_size
						);
				if (!into_wall) {
					break;
				}
			}
			p.position = to;

			// take into account skin width of nearby solid cells
			vec3d grid_pos = p.position - grid_offset;
			vec3s cell_index(grid_pos / cell_size);
			vec3d cell_pos = grid_pos - vec3d(cell_index) * cell_size;
			double cell_skin_max = cell_size - boundary_skin_width;
			vec_ops::for_each(
				[this, grid_pos, cell_index, cell_skin_max](double cp, double &pos, std::size_t dim) {
					vec3s offset;
					offset[dim] = 1;
					if (cp < boundary_skin_width) {
						if (
							cell_index[dim] == 0 ||
							grid().grid()(cell_index - offset).cell_type == mac_grid::cell::type::solid
						) {
							pos += boundary_skin_width - cp;
						}
					}
					if (cp > cell_skin_max) {
						if (
							cell_index[dim] + 1 >= grid().grid().get_size()[dim] ||
							grid().grid()(cell_index + offset).cell_type == mac_grid::cell::type::solid
						) {
							pos += cell_skin_max - cp;
						}
					}
				},
				cell_pos, p.position, vec3s(0, 1, 2)
					);
		}
	}

	void simulation::_extrapolate_velocities(const std::vector<vec3s> &fluid_cells) {
		grid3<unsigned char> valid(grid().grid().get_size(), 0);
		for (vec3s cell : fluid_cells) {
			valid(cell) = 1;
		}

		std::vector<std::size_t> new_valid_cells;

		for (std::size_t i = 0; i < 1; ++i) {
			// update valid cells
			for (std::size_t cell : new_valid_cells) {
				valid[cell] = 1;
			}
			new_valid_cells.clear();

			// one iteration of extrapolation
			valid.for_each(
				[this, &valid, &new_valid_cells](vec3s pos, unsigned char cell_valid) {
					if (cell_valid != 0) {
						return;
					}
					std::size_t valid_neighbors = 0;
					vec3d neighbor_vels;
					vec3<mac_grid::cell::type> type_pos(
						mac_grid::cell::type::solid,
						mac_grid::cell::type::solid,
						mac_grid::cell::type::solid
					);
					std::size_t pos_flat = grid().grid().index_to_raw(pos);
					mac_grid::cell &this_cell = grid().grid()[pos_flat];
					vec_ops::for_each(
						[&](std::size_t dim) {
							if (pos[dim] > 0) {
								vec3s neg = pos;
								--neg[dim];
								if (valid(neg) != 0) {
									const mac_grid::cell &neg_cell = grid().grid()(neg);
									neighbor_vels += neg_cell.velocities_posface;
									++valid_neighbors;
								}
							}
							if (pos[dim] + 1 < grid().grid().get_size()[dim]) {
								vec3s pospos = pos;
								++pospos[dim];
								if (valid(pospos) != 0) {
									const mac_grid::cell &pos_cell = grid().grid()(pospos);
									neighbor_vels += pos_cell.velocities_posface;
									type_pos[dim] = pos_cell.cell_type;
									++valid_neighbors;
								}
							}
						},
						vec3s(0, 1, 2)
							);
					if (valid_neighbors > 0) {
						vec_ops::for_each(
							[&](std::size_t dim) {
								if (this_cell.cell_type == type_pos[dim]) {
									this_cell.velocities_posface[dim] =
										neighbor_vels[dim] / static_cast<double>(valid_neighbors);
								}
							},
							vec3s(0, 1, 2)
								);
						new_valid_cells.emplace_back(pos_flat);
					}
				}
			);
		}
	}

	void simulation::_update_sources() {
		for (auto &src : sources) {
			if (!src->active) {
				continue;
			}
			for (vec3s v : src->cells) {
				seed_cell(v, src->velocity, src->target_density_cubic_root);
			}
		}
	}
}
