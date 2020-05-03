#include "test_scenes.h"

#include <fluid/math/constants.h>
#include <fluid/renderer/camera.h>
#include <fluid/renderer/material.h>

using namespace fluid;
using namespace fluid::renderer;

scene::mesh_t create_plane() {
	scene::mesh_t mesh;

	mesh.positions.emplace_back(-0.5, 0.0, -0.5);
	mesh.positions.emplace_back(0.5, 0.0, -0.5);
	mesh.positions.emplace_back(0.5, 0.0, 0.5);
	mesh.positions.emplace_back(-0.5, 0.0, 0.5);

	mesh.indices.emplace_back(0);
	mesh.indices.emplace_back(1);
	mesh.indices.emplace_back(2);

	mesh.indices.emplace_back(0);
	mesh.indices.emplace_back(2);
	mesh.indices.emplace_back(3);

	return mesh;
}

scene::mesh_t create_box() {
	scene::mesh_t mesh;

	mesh.positions.emplace_back(-0.5, -0.5, -0.5);
	mesh.positions.emplace_back(0.5, -0.5, -0.5);
	mesh.positions.emplace_back(0.5, 0.5, -0.5);
	mesh.positions.emplace_back(-0.5, 0.5, -0.5);
	mesh.positions.emplace_back(-0.5, -0.5, 0.5);
	mesh.positions.emplace_back(0.5, -0.5, 0.5);
	mesh.positions.emplace_back(0.5, 0.5, 0.5);
	mesh.positions.emplace_back(-0.5, 0.5, 0.5);

	mesh.indices = std::vector<std::size_t>(
		{
			0, 1, 3, 3, 1, 2,
			1, 5, 2, 2, 5, 6,
			5, 4, 6, 6, 4, 7,
			4, 0, 7, 7, 0, 3,
			3, 2, 7, 7, 2, 6,
			4, 5, 0, 0, 5, 1
		}
	);

	return mesh;
}


std::pair<scene, camera> cornell_box() {
	scene result;

	material matte_white;
	{
		auto &lambert = matte_white.value.emplace<materials::lambertian_reflection>();
		lambert.reflectance.modulation = spectrum(vec3d(0.725, 0.71, 0.68));
	}

	material matte_red;
	{
		auto &lambert = matte_red.value.emplace<materials::lambertian_reflection>();
		lambert.reflectance.modulation = spectrum(vec3d(0.63, 0.065, 0.05));
	}

	material matte_green;
	{
		auto &lambert = matte_green.value.emplace<materials::lambertian_reflection>();
		lambert.reflectance.modulation = spectrum(vec3d(0.14, 0.45, 0.091));
	}

	scene::mesh_t plane = create_plane(), box = create_box();

	entity_info floor;
	floor.mat = matte_white;
	result.add_mesh_entity(
		plane,
		transform::scale_rotate_translate(
			vec3d(10.0, 1.0, 10.0), vec3d(constants::pi, 0.0, 0.0), vec3d(0.0, -2.5, 0.0)
		),
		floor
	);

	entity_info left_wall;
	left_wall.mat = matte_red;
	result.add_mesh_entity(
		plane,
		transform::scale_rotate_translate(
			vec3d(10.0, 1.0, 10.0), vec3d(0.0, 0.0, -0.5 * constants::pi), vec3d(5.0, 2.5, 0.0)
		),
		left_wall
	);

	entity_info right_wall;
	right_wall.mat = matte_green;
	result.add_mesh_entity(
		plane,
		transform::scale_rotate_translate(
			vec3d(10.0, 1.0, 10.0), vec3d(0.0, 0.0, 0.5 * constants::pi), vec3d(-5.0, 2.5, 0.0)
		),
		right_wall
	);

	entity_info back_wall;
	back_wall.mat = matte_white;
	result.add_mesh_entity(
		plane,
		transform::scale_rotate_translate(
			vec3d(10.0, 1.0, 10.0), vec3d(-0.5 * constants::pi, 0.0, 0.0), vec3d(0.0, 2.5, 5.0)
		),
		back_wall
	);

	entity_info ceiling;
	ceiling.mat = matte_white;
	result.add_mesh_entity(
		plane,
		transform::scale_rotate_translate(
			vec3d(10.0, 1.0, 10.0), vec3d(0.0, 0.0, 0.0), vec3d(0.0, 7.5, 0.0)
		),
		ceiling
	);

	entity_info long_cube;
	long_cube.mat = matte_white;
	result.add_mesh_entity(
		box,
		transform::scale_rotate_translate(
			vec3d(3.0, 6.0, 3.0), vec3d(0.0, 27.5 * constants::pi / 180.0, 0.0), vec3d(2.0, 0.0, 3.0)
		),
		long_cube
	);

	entity_info short_cube;
	short_cube.mat = matte_white;
	result.add_mesh_entity(
		box,
		transform::scale_rotate_translate(
			vec3d(3.0, 3.0, 3.0), vec3d(0.0, -17.5 * constants::pi / 180.0, 0.0), vec3d(-2.0, -1.0, 0.75)
		),
		short_cube
	);

	entity_info light;
	light.mat = matte_white;
	light.mat.emission.modulation = spectrum(2.0 * vec3d(17.0, 12.0, 4.0));
	result.add_mesh_entity(
		plane,
		transform::scale_rotate_translate(
			vec3d(3.0, 1.0, 3.0), vec3d(0.0, 0.0, 0.0), vec3d(0.0, 7.45, 0.0)
		),
		light
	);

	camera cam = camera::from_parameters(
		vec3d(0.0, 5.5, -30.0), vec3d(0.0, 2.5, 0.0), vec3d(0.0, 1.0, 0.0),
		19.5 * constants::pi / 180.0, 1.0
	);

	result.finish();
	return { std::move(result), cam };
}
