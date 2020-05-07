#include <iostream>
#include <fstream>
#include <atomic>
#include <thread>
#include <mutex>

#ifdef _WIN32
#	define NOMINMAX
#	include <Windows.h>
#endif
#include <GL/GL.h>
#include <GL/GLU.h>
#include <GLFW/glfw3.h>

#include <fluid/simulation.h>
#include <fluid/data_structures/grid.h>
#include <fluid/mesher.h>
#include <fluid/data_structures/point_cloud.h>

#include <fluid/renderer/path_tracer.h>

#include "test_scenes.h"

using fluid::vec2d;
using fluid::vec2s;
using fluid::vec3d;
using fluid::vec3s;

using namespace fluid::renderer;

#define USE_OBSTACLE 1

std::atomic_bool
sim_paused = true,
sim_reset = true,
sim_advance = false;
vec3d sim_grid_offset;
vec3s sim_grid_size(50, 50, 50);
double sim_cell_size = 1.0;
std::mutex sim_particles_lock;
std::vector<fluid::simulation::particle> sim_particles;
fluid::grid3<std::size_t> sim_grid_occupation;
fluid::grid3<vec3d> sim_grid_velocities;
bool sim_mesh_valid = false;

fluid::semaphore sim_mesher_sema;

void update_simulation(const fluid::simulation &sim) {
	// collect particles
	std::vector<fluid::simulation::particle> new_particles(sim.particles().begin(), sim.particles().end());

	double energy = 0.0;
	for (fluid::simulation::particle &p : new_particles) {
		energy += 0.5 * p.velocity.squared_length();
		energy -= fluid::vec_ops::dot(sim.gravity, p.position);
	}
	std::cout << "    total energy: " << energy << "\n";

	// collect occupation
	fluid::grid3<std::size_t> grid(sim.grid().grid().get_size(), 0);
	for (const auto &particle : sim.particles()) {
		vec3s pos(fluid::vec3i((particle.position - sim.grid_offset) / sim.cell_size));
		if (pos.x < grid.get_size().x && pos.y < grid.get_size().y && pos.z < grid.get_size().z) {
			++grid(pos);
		}
	}

	// collect velocities
	fluid::grid3<vec3d> grid_vels(sim.grid().grid().get_size());
	for (std::size_t z = 0; z < grid_vels.get_size().z; ++z) {
		for (std::size_t y = 0; y < grid_vels.get_size().y; ++y) {
			for (std::size_t x = 0; x < grid_vels.get_size().x; ++x) {
				grid_vels(x, y, z) = sim.grid().grid()(x, y, z).velocities_posface;
			}
		}
	}

	{
		std::lock_guard<std::mutex> guard(sim_particles_lock);
		sim_particles = std::move(new_particles);
		sim_grid_occupation = std::move(grid);
		sim_grid_velocities = std::move(grid_vels);
		sim_mesh_valid = false;
		sim_mesher_sema.notify();
	}
}

void simulation_thread() {
	fluid::simulation sim;

	sim.resize(sim_grid_size);
	sim.grid_offset = sim_grid_offset;
	sim.cell_size = sim_cell_size;
	sim.simulation_method = fluid::simulation::method::apic;
	/*sim.blending_factor = 0.99;*/
	sim.blending_factor = 1.0;
	sim.gravity = vec3d(0.0, -981.0, 0.0);

	sim.pre_time_step_callback = [](double dt) {
		std::cout << "  time step " << dt << "\n";
	};
	sim.post_pressure_solve_callback = [&sim](
		double, std::vector<double> &pressure, double residual, std::size_t iters
	) {
		std::cout << "    iterations = " << iters << "\n";
		if (iters > 100) {
			std::cout << "*** WARNING: large number of iterations\n";
		}
		std::cout << "    residual = " << residual << "\n";
		auto max_it = std::max_element(pressure.begin(), pressure.end());
		if (max_it != pressure.end()) {
			std::cout << "    max pressure = " << *max_it << "\n";
		}
	};
	sim.post_grid_to_particle_transfer_callback = [&sim](double) {
		double maxv = 0.0;
		for (const fluid::simulation::particle &p : sim.particles()) {
			maxv = std::max(maxv, p.velocity.squared_length());
		}
		std::cout << "    max particle velocity = " << std::sqrt(maxv) << "\n";
	};

	{
		auto source = std::make_unique<fluid::source>();

		for (std::size_t x = 20; x < 30; ++x) {
			for (std::size_t y = 40; y < 45; ++y) {
				for (std::size_t z = 20; z < 30; ++z) {
					source->cells.emplace_back(x, y, z);
				}
			}
		}

		/*for (std::size_t x = 5; x < 10; ++x) {
			for (std::size_t y = 30; y < 40; ++y) {
				for (std::size_t z = 20; z < 30; ++z) {
					source->cells.emplace_back(x, y, z);
				}
			}
		}
		source->velocity = vec3d(200.0, 0.0, 0.0);
		source->coerce_velocity = true;*/

		sim.sources.emplace_back(std::move(source));
	}

	{
#if USE_OBSTACLE
		sim.grid().grid().for_each(
			[](vec3s, fluid::mac_grid::cell &cell) {
				cell.cell_type = fluid::mac_grid::cell::type::solid;
			}
		);
		sim.grid().grid().for_each_in_range_checked(
			[](vec3s, fluid::mac_grid::cell &cell) {
				cell.cell_type = fluid::mac_grid::cell::type::air;
			},
			vec3s(1, 1, 1), vec3s(49, 49, 49)
				);
#endif
	}

	while (true) {
		if (sim_reset) {
			sim.particles().clear();

			/*sim.seed_box(vec3d(20, 20, 20), vec3d(10, 10, 10));*/
			/*sim.seed_box(vec3d(15, 15, 15), vec3d(20, 20, 20));*/
			/*sim.seed_box(vec3d(10, 10, 10), vec3d(30, 30, 30));*/
			/*sim.seed_sphere(vec3d(25.0, 25.0, 25.0), 10.0);*/
			/*sim.seed_sphere(vec3d(25.0, 25.0, 25.0), 15.0);*/

			/*sim.seed_box(vec3d(0, 0, 0), vec3d(10, 50, 50));*/

			/*sim.seed_sphere(vec3d(25, 40, 25), 5);
			sim.seed_box(vec3d(0, 0, 0), vec3d(50, 15, 50));*/

			sim.reset_space_hash();

			update_simulation(sim);
			sim_reset = false;
		}

		if (!sim_paused) {
			std::cout << "update\n";
			sim.update(1.0 / 60.0);
			update_simulation(sim);
		} else if (sim_advance) {
			sim_advance = false;
			sim.time_step();
			update_simulation(sim);
		}
	}
}


std::mutex sim_mesh_lock;
fluid::mesher::mesh_t sim_mesh;

void mesher_thread() {
	while (true) {
		sim_mesher_sema.wait();
		std::vector<vec3d> particles;
		{
			std::lock_guard<std::mutex> lock(sim_particles_lock);
			if (sim_mesh_valid) {
				continue;
			}
			for (const fluid::simulation::particle &p : sim_particles) {
				particles.emplace_back(p.position);
			}
			sim_mesh_valid = true;
		}

		fluid::mesher mesher;
		mesher.particle_extent = 2.0; // TODO values close or smaller than 1 causes holes to appear in meshes
		mesher.cell_radius = 3;
		mesher.grid_offset = vec3d(-1.0, -1.0, -1.0);
		mesher.cell_size = 0.5;
		mesher.resize(vec3s(104, 104, 104));
		fluid::mesher::mesh_t mesh = mesher.generate_mesh(particles, 0.5);
		mesh.generate_normals();

		{
			std::lock_guard<std::mutex> lock(sim_mesh_lock);
			sim_mesh = std::move(mesh);
		}
	}
}


enum class particle_visualize_mode : unsigned char {
	none,
	velocity_direction,
	velocity_magnitude,
	maximum
};
enum class mesh_visualize_mode : unsigned char {
	none,
	transparent,
	maximum
};

bool
rotating = false,
draw_particles = true,
draw_cells = false,
draw_faces = false,
draw_mesh = true,
draw_apic_debug = false;
vec2d mouse, rotation;
particle_visualize_mode particle_vis = particle_visualize_mode::none;
mesh_visualize_mode mesh_vis = mesh_visualize_mode::none;
double camera_distance = -70.0;

// callbacks
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	if (action == GLFW_PRESS) {
		switch (key) {
		case GLFW_KEY_ENTER:
			sim_paused = !sim_paused;
			break;
		case GLFW_KEY_R:
			sim_reset = true;
			break;
		case GLFW_KEY_SPACE:
			sim_advance = true;
			break;

		case GLFW_KEY_P:
			draw_particles = !draw_particles;
			break;
		case GLFW_KEY_C:
			draw_cells = !draw_cells;
			break;
		case GLFW_KEY_F:
			draw_faces = !draw_faces;
			break;
		case GLFW_KEY_M:
			draw_mesh = !draw_mesh;
			break;
		case GLFW_KEY_A:
			draw_apic_debug = !draw_apic_debug;
			break;

		case GLFW_KEY_F1:
			particle_vis = static_cast<particle_visualize_mode>(static_cast<unsigned char>(particle_vis) + 1);
			if (particle_vis == particle_visualize_mode::maximum) {
				particle_vis = particle_visualize_mode::none;
			}
			break;
		case GLFW_KEY_F2:
			mesh_vis = static_cast<mesh_visualize_mode>(static_cast<unsigned char>(mesh_vis) + 1);
			if (mesh_vis == mesh_visualize_mode::maximum) {
				mesh_vis = mesh_visualize_mode::none;
			}
			break;

		case GLFW_KEY_F5:
			{
				std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
				/*auto [scene, cam] = cornell_box();*/
				auto [scene, cam] = glass_ball_box();
				scene.finish();
				std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
				path_tracer tracer;
				auto img = tracer.render(scene, cam, vec2s(400, 400));
				img.save_ppm(
					"test.ppm",
					[](spectrum pixel) {
						vec3d rgb = pixel.to_rgb() * 255.0;
						return fluid::vec_ops::apply<fluid::vec3<std::uint8_t>>(
							[](double v) {
								return static_cast<std::uint8_t>(std::clamp(v, 0.0, 255.0));
							},
							rgb
								);
					}
				);
				std::chrono::high_resolution_clock::time_point t3 = std::chrono::high_resolution_clock::now();
				std::cout << "build scene: " << std::chrono::duration<double, std::milli>(t2 - t1).count() << "ms\n";
				std::cout << "render: " << std::chrono::duration<double, std::milli>(t3 - t2).count() << "ms\n";
			}
			break;

		case GLFW_KEY_S:
			{
				std::lock_guard<std::mutex> guard(sim_mesh_lock);
				std::ofstream fout("mesh.obj");
				sim_mesh.save_obj(fout);
			}
			break;
		case GLFW_KEY_D:
			{
				std::vector<vec3d> points;
				{
					std::lock_guard<std::mutex> guard(sim_particles_lock);
					for (const fluid::simulation::particle &p : sim_particles) {
						points.emplace_back(p.position);
					}
				}
				std::ofstream fout("points.txt");
				fluid::point_cloud::save_to_naive(fout, points.begin(), points.end());
			}
			break;
		}
	}
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
	if (button == GLFW_MOUSE_BUTTON_LEFT) {
		rotating = action == GLFW_PRESS;
	}
}

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
	vec2d new_mouse(xpos, ypos);
	if (rotating) {
		rotation += new_mouse - mouse;
		rotation.y = std::clamp(rotation.y, -90.0, 90.0);
	}
	mouse = new_mouse;
}

void resize_callback(GLFWwindow *window, int width, int height) {
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, width / static_cast<double>(height), 0.1, 1000.0);
}

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
	camera_distance += yoffset;
}


int main() {
	if (!glfwInit()) {
		return -1;
	}

	GLFWwindow *window = glfwCreateWindow(800, 600, "libfluid", nullptr, nullptr);
	if (!window) {
		glfwTerminate();
		return -1;
	}
	glfwSetKeyCallback(window, key_callback);
	glfwSetMouseButtonCallback(window, mouse_button_callback);
	glfwSetCursorPosCallback(window, cursor_position_callback);
	glfwSetWindowSizeCallback(window, resize_callback);
	glfwSetScrollCallback(window, scroll_callback);

	glfwMakeContextCurrent(window);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	int width, height;
	glfwGetWindowSize(window, &width, &height);
	resize_callback(window, width, height);

	std::thread sim_thread(simulation_thread);
	sim_thread.detach();
	std::thread mesh_thread(mesher_thread);
	mesh_thread.detach();

	while (!glfwWindowShouldClose(window)) {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glTranslated(0, 0, camera_distance);
		glRotated(rotation.y, 1, 0, 0);
		glRotated(rotation.x, 0, 1, 0);
		glTranslatef(-25, -25, -25);

		glEnable(GL_DEPTH_TEST);

		glBegin(GL_LINES);
		vec3d
			min_corner = sim_grid_offset,
			max_corner = sim_grid_offset + vec3d(sim_grid_size) * sim_cell_size;

		glColor3d(0.5, 0.0, 0.0);
		glVertex3d(min_corner.x, min_corner.y, min_corner.z);
		glVertex3d(max_corner.x, min_corner.y, min_corner.z);

		glColor3d(0.0, 0.5, 0.0);
		glVertex3d(min_corner.x, min_corner.y, min_corner.z);
		glVertex3d(min_corner.x, max_corner.y, min_corner.z);

		glColor3d(0.0, 0.0, 0.5);
		glVertex3d(min_corner.x, min_corner.y, min_corner.z);
		glVertex3d(min_corner.x, min_corner.y, max_corner.z);

		glColor3d(0.3, 0.3, 0.3);

		glVertex3d(max_corner.x, min_corner.y, min_corner.z);
		glVertex3d(max_corner.x, max_corner.y, min_corner.z);

		glVertex3d(max_corner.x, min_corner.y, min_corner.z);
		glVertex3d(max_corner.x, min_corner.y, max_corner.z);

		glVertex3d(min_corner.x, max_corner.y, min_corner.z);
		glVertex3d(max_corner.x, max_corner.y, min_corner.z);

		glVertex3d(min_corner.x, max_corner.y, min_corner.z);
		glVertex3d(min_corner.x, max_corner.y, max_corner.z);

		glVertex3d(min_corner.x, min_corner.y, max_corner.z);
		glVertex3d(max_corner.x, min_corner.y, max_corner.z);

		glVertex3d(min_corner.x, min_corner.y, max_corner.z);
		glVertex3d(min_corner.x, max_corner.y, max_corner.z);

		glVertex3d(max_corner.x, max_corner.y, min_corner.z);
		glVertex3d(max_corner.x, max_corner.y, max_corner.z);

		glVertex3d(max_corner.x, min_corner.y, max_corner.z);
		glVertex3d(max_corner.x, max_corner.y, max_corner.z);

		glVertex3d(min_corner.x, max_corner.y, max_corner.z);
		glVertex3d(max_corner.x, max_corner.y, max_corner.z);

		glEnd();

		if (draw_mesh) {
			glEnable(GL_CULL_FACE);
			glCullFace(GL_FRONT);

			glEnable(GL_LIGHTING);

			glEnable(GL_LIGHT0);
			GLfloat color1[]{ 0.6, 0.6, 0.6, 1 };
			glLightfv(GL_LIGHT0, GL_DIFFUSE, color1);
			glLightfv(GL_LIGHT0, GL_SPECULAR, color1);
			GLfloat dir1[]{ -1, -1, -1, 0 };
			glLightfv(GL_LIGHT0, GL_POSITION, dir1);

			glEnable(GL_LIGHT1);
			GLfloat color2[]{ 0.6, 0.6, 0.6, 1 };
			glLightfv(GL_LIGHT1, GL_DIFFUSE, color2);
			glLightfv(GL_LIGHT1, GL_SPECULAR, color2);
			GLfloat dir2[]{ 1, -1, 1, 0 };
			glLightfv(GL_LIGHT1, GL_POSITION, dir2);

			glEnable(GL_COLOR_MATERIAL);
			glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT | GL_DIFFUSE);

			glBegin(GL_TRIANGLES);
			switch (mesh_vis) {
			case mesh_visualize_mode::none:
				glColor4d(1.0, 1.0, 1.0, 1.0);
				break;
			case mesh_visualize_mode::transparent:
				glColor4d(0.5, 0.5, 1.0, 0.2);
				break;
			}
			{
				std::lock_guard<std::mutex> guard(sim_mesh_lock);
				for (std::size_t i = 0; i < sim_mesh.indices.size(); ++i) {
					vec3d v = sim_mesh.positions[sim_mesh.indices[i]];
					vec3d n = sim_mesh.normals[sim_mesh.indices[i]];
					glNormal3d(n.x, n.y, n.z);
					glVertex3d(v.x, v.y, v.z);
				}
			}
			glEnd();

			glDisable(GL_LIGHTING);
		}

		glDisable(GL_DEPTH_TEST);

		if (draw_faces) {
			glBegin(GL_LINES);
			glColor3d(1.0, 0.0, 0.0);
			double half_cell = 0.5 * sim_cell_size;
			{
				std::lock_guard<std::mutex> guard(sim_particles_lock);
				for (std::size_t z = 0; z < sim_grid_velocities.get_size().z; ++z) {
					for (std::size_t y = 0; y < sim_grid_velocities.get_size().y; ++y) {
						for (std::size_t x = 0; x < sim_grid_velocities.get_size().x; ++x) {
							vec3d
								face_outer = sim_grid_offset + vec3d(vec3s(x + 1, y + 1, z + 1)) * sim_cell_size,
								face_half = face_outer - vec3d(half_cell, half_cell, half_cell),
								vel = sim_grid_velocities(x, y, z) * 0.001;

							if (vel.x > 0.0) {
								glColor4d(1.0, 0.0, 0.0, 0.3);
							} else {
								glColor4d(0.0, 1.0, 1.0, 0.3);
							}
							glVertex3d(face_outer.x, face_half.y, face_half.z);
							glVertex3d(face_outer.x + vel.x, face_half.y, face_half.z);

							if (vel.y > 0.0) {
								glColor4d(0.0, 1.0, 0.0, 0.3);
							} else {
								glColor4d(1.0, 0.0, 1.0, 0.3);
							}
							glVertex3d(face_half.x, face_outer.y, face_half.z);
							glVertex3d(face_half.x, face_outer.y + vel.y, face_half.z);

							if (vel.z > 0.0) {
								glColor4d(0.0, 0.0, 1.0, 0.3);
							} else {
								glColor4d(1.0, 1.0, 0.0, 0.3);
							}
							glVertex3d(face_half.x, face_half.y, face_outer.z);
							glVertex3d(face_half.x, face_half.y, face_outer.z + vel.z);
						}
					}
				}
			}
			glEnd();
		}

		if (draw_particles) {
			glBegin(GL_POINTS);
			{
				std::lock_guard<std::mutex> guard(sim_particles_lock);

				double max_vel = 0.0;
				for (fluid::simulation::particle &p : sim_particles) {
					max_vel = std::max(max_vel, p.velocity.squared_length());
				}
				max_vel = std::sqrt(max_vel);

				for (fluid::simulation::particle &p : sim_particles) {
					vec3d pos = p.position;
					switch (particle_vis) {
					case particle_visualize_mode::none:
						glColor4d(1.0, 1.0, 1.0, 0.3);
						break;
					case particle_visualize_mode::velocity_direction:
						{
							vec3d vel = fluid::vec_ops::apply<vec3d>(
								[](double v) {
									v /= 1.0;
									if (v > -1.0 && v < 1.0) {
										v = v < 0.0 ? -1.0 : 1.0;
									}
									return std::clamp(std::log(v) + 0.5, 0.0, 1.0);
								}, p.velocity
							);
							glColor4d(vel.x, vel.y, vel.z, 0.3);
						}
						break;
					case particle_visualize_mode::velocity_magnitude:
						{
							double c = p.velocity.length() / max_vel;
							glColor4d(1.0, 1.0, 1.0, c * 0.9 + 0.1);
						}
						break;
					default:
						glColor4d(1.0, 0.0, 0.0, 1.0);
						break;
					}
					glVertex3d(pos.x, pos.y, pos.z);
				}
			}
			glEnd();
		}

		if (draw_apic_debug) {
			glBegin(GL_LINES);
			{
				std::lock_guard<std::mutex> guard(sim_particles_lock);
				for (fluid::simulation::particle &p : sim_particles) {
					double mul = 0.01;
					vec3d
						pos = p.position,
						pcx = pos + p.cx * mul,
						pcy = pos + p.cy * mul,
						pcz = pos + p.cz * mul;

					glColor4d(1.0, 0.0, 0.0, 1.0);
					glVertex3d(pos.x, pos.y, pos.z);
					glVertex3d(pcx.x, pcx.y, pcx.z);

					glColor4d(0.0, 1.0, 0.0, 1.0);
					glVertex3d(pos.x, pos.y, pos.z);
					glVertex3d(pcy.x, pcy.y, pcy.z);

					glColor4d(0.0, 0.0, 1.0, 1.0);
					glVertex3d(pos.x, pos.y, pos.z);
					glVertex3d(pcz.x, pcz.y, pcz.z);
				}
			}
			glEnd();
		}

		if (draw_cells) {
			glBegin(GL_POINTS);
			glColor4d(0.0, 1.0, 0.0, 1.0);
			{
				std::lock_guard<std::mutex> guard(sim_particles_lock);
				for (std::size_t z = 0; z < sim_grid_occupation.get_size().z; ++z) {
					for (std::size_t y = 0; y < sim_grid_occupation.get_size().y; ++y) {
						for (std::size_t x = 0; x < sim_grid_occupation.get_size().x; ++x) {
							if (sim_grid_occupation(x, y, z) > 0) {
								vec3d pos = sim_grid_offset + sim_cell_size * (vec3d(vec3s(x, y, z)) + vec3d(0.5, 0.5, 0.5));
								glVertex3d(pos.x, pos.y, pos.z);
							}
						}
					}
				}
			}
			glEnd();
		}

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwTerminate();
	return 0;
}
