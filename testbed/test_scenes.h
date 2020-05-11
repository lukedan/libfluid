#pragma once

#include <fluid/renderer/scene.h>
#include <fluid/renderer/camera.h>

fluid::renderer::scene::mesh_t create_plane();
fluid::renderer::scene::mesh_t create_box();

std::pair<fluid::renderer::scene, fluid::renderer::camera> red_green_box();
std::pair<fluid::renderer::scene, fluid::renderer::camera> cornell_box_base();
std::pair<fluid::renderer::scene, fluid::renderer::camera> cornell_box_one_light();
std::pair<fluid::renderer::scene, fluid::renderer::camera> cornell_box_two_lights();
std::pair<fluid::renderer::scene, fluid::renderer::camera> glass_ball_box();

std::pair<fluid::renderer::scene, fluid::renderer::camera> fluid_box(fluid::vec3d min, fluid::vec3d max);