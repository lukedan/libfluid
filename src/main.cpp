#include <iostream>
#include <atomic>
#include <thread>
#include <mutex>

#ifdef _WIN32
#	define NOMINMAX
#	include <Windows.h>
#endif
#include <GL\GL.h>
#include <GL\GLU.h>
#include <GLFW\glfw3.h>

#include "simulation.h"

using fluid::vec2d;
using fluid::vec3d;
using fluid::vec3s;

std::atomic_bool paused = true;

bool rotating = false;
vec2d mouse, rotation;


// callbacks
void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {
	if (key == GLFW_KEY_ENTER && action == GLFW_PRESS) {
		paused = !paused;
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
	}
	mouse = new_mouse;
}

void resize_callback(GLFWwindow *window, int width, int height) {
	glViewport(0, 0, width, height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, width / static_cast<double>(height), 0.1, 1000.0);
}


vec3d sim_grid_offset;
vec3s sim_grid_size(50, 50, 50);
double sim_cell_size = 1.0;
std::mutex sim_particles_lock;
std::vector<vec3d> sim_particles;

void update_particles(const std::vector<fluid::particle> &particles) {
	std::lock_guard<std::mutex> guard(sim_particles_lock);
	sim_particles.clear();
	for (const auto &particle : particles) {
		sim_particles.emplace_back(particle.position);
	}
}

void worker_thread() {
	fluid::simulation sim;

	sim.resize(sim_grid_size);
	sim.grid_offset = sim_grid_offset;
	sim.cell_size = sim_cell_size;

	/*sim.seed_box(fluid::vec3s(20, 20, 20), fluid::vec3s(10, 10, 10));*/
	sim.seed_box(fluid::vec3s(15, 15, 15), fluid::vec3s(20, 20, 20));
	/*sim.seed_box(fluid::vec3s(10, 10, 10), fluid::vec3s(30, 30, 30));*/
	/*sim.seed_box(vec3s(0, 0, 0), vec3s(10, 50, 50));*/
	update_particles(sim.get_particles());

	while (true) {
		if (!paused) {
			sim.update(1.0 / 30.0);
			update_particles(sim.get_particles());
		}
	}
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

	glfwMakeContextCurrent(window);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	int width, height;
	glfwGetWindowSize(window, &width, &height);
	resize_callback(window, width, height);

	std::thread t(worker_thread);
	t.detach();

	while (!glfwWindowShouldClose(window)) {
		glClear(GL_COLOR_BUFFER_BIT);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glTranslatef(0, 0, -75);
		glRotated(rotation.y, 1, 0, 0);
		glRotated(rotation.x, 0, 1, 0);
		glTranslatef(-25, -25, -25);

		glBegin(GL_LINES);
		vec3d
			min_corner = sim_grid_offset,
			max_corner = sim_grid_offset + vec3d(sim_grid_size) * sim_cell_size;
		glColor3d(0.3, 0.3, 0.3);

		glVertex3d(min_corner.x, min_corner.y, min_corner.z);
		glVertex3d(max_corner.x, min_corner.y, min_corner.z);

		glVertex3d(min_corner.x, min_corner.y, min_corner.z);
		glVertex3d(min_corner.x, max_corner.y, min_corner.z);

		glVertex3d(min_corner.x, min_corner.y, min_corner.z);
		glVertex3d(min_corner.x, min_corner.y, max_corner.z);

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

		// face velocity indicators - very ugly and unuseful
		/*glBegin(GL_LINES);
		glColor3d(1.0, 0.0, 0.0);
		double half_cell = 0.5 * sim.cell_size;
		for (std::size_t z = 0; z < sim.grid().grid().get_size().z; ++z) {
			for (std::size_t y = 0; y < sim.grid().grid().get_size().y; ++y) {
				for (std::size_t x = 0; x < sim.grid().grid().get_size().x; ++x) {
					fluid::vec3d
						face_outer = sim.grid_offset + fluid::vec3d(x + 1, y + 1, z + 1) * sim.cell_size,
						face_half = face_outer - fluid::vec3d(half_cell, half_cell, half_cell),
						vel = sim.grid().grid()(x, y, z).velocities_posface * 0.001;

					glVertex3d(face_outer.x, face_half.y, face_half.z);
					glVertex3d(face_outer.x + vel.x, face_half.y, face_half.z);

					glVertex3d(face_half.x, face_outer.y, face_half.z);
					glVertex3d(face_half.x, face_outer.y + vel.y, face_half.z);

					glVertex3d(face_half.x, face_half.y, face_outer.z);
					glVertex3d(face_half.x, face_half.y, face_outer.z + vel.z);
				}
			}
		}
		glEnd();*/

		glBegin(GL_POINTS);
		glColor4d(1.0, 1.0, 1.0, 0.2);
		{
			std::lock_guard<std::mutex> guard(sim_particles_lock);
			for (vec3d pos : sim_particles) {
				glVertex3d(pos.x, pos.y, pos.z);
			}
		}
		glEnd();

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glfwTerminate();
	return 0;
}
