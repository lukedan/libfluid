#pragma once

#include <fluid/renderer/scene.h>
#include <fluid/renderer/camera.h>

fluid::renderer::scene::mesh_t create_plane();
fluid::renderer::scene::mesh_t create_box();

std::pair<fluid::renderer::scene, fluid::renderer::camera> red_green_box(double asp_ratio);
std::pair<fluid::renderer::scene, fluid::renderer::camera> cornell_box_base(double asp_ratio);
std::pair<fluid::renderer::scene, fluid::renderer::camera> cornell_box_one_light(double asp_ratio);
std::pair<fluid::renderer::scene, fluid::renderer::camera> cornell_box_two_lights(double asp_ratio);
std::pair<fluid::renderer::scene, fluid::renderer::camera> glass_ball_box(double asp_ratio);

std::pair<fluid::renderer::scene, fluid::renderer::camera> fluid_box(
	fluid::vec3d min, fluid::vec3d max, double fovy, double asp_ratio
);