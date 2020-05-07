#pragma once

#include <fluid/renderer/scene.h>
#include <fluid/renderer/camera.h>

fluid::renderer::scene::mesh_t create_plane();
fluid::renderer::scene::mesh_t create_box();

std::pair<fluid::renderer::scene, fluid::renderer::camera> cornell_box();
std::pair<fluid::renderer::scene, fluid::renderer::camera> glass_ball_box();