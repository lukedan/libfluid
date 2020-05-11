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
			0, 3, 1, 3, 2, 1,
			1, 2, 5, 2, 6, 5,
			5, 6, 4, 6, 7, 4,
			4, 7, 0, 7, 3, 0,
			3, 7, 2, 7, 6, 2,
			4, 0, 5, 0, 1, 5
		}
	);

	return mesh;
}


std::pair<scene, camera> red_green_box() {
	scene result;

	material matte_white;
	{
		auto &lambert = matte_white.value.emplace<materials::lambertian_reflection>();
		lambert.reflectance.modulation = spectrum::from_rgb(vec3d(0.725, 0.71, 0.68));
	}

	material matte_red;
	{
		auto &lambert = matte_red.value.emplace<materials::lambertian_reflection>();
		lambert.reflectance.modulation = spectrum::from_rgb(vec3d(0.63, 0.065, 0.05));
	}

	material matte_green;
	{
		auto &lambert = matte_green.value.emplace<materials::lambertian_reflection>();
		lambert.reflectance.modulation = spectrum::from_rgb(vec3d(0.14, 0.45, 0.091));
	}

	material mirror;
	{
		auto &specular = mirror.value.emplace<materials::specular_reflection>();
		specular.reflectance.modulation = spectrum::identity;
	}

	scene::mesh_t plane = create_plane();

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
	/*back_wall.mat = mirror;*/
	back_wall.mat = matte_white;
	result.add_mesh_entity(
		plane,
		transform::scale_rotate_translate(
			vec3d(10.0, 1.0, 10.0), vec3d(0.5 * constants::pi, 0.0, 0.0), vec3d(0.0, 2.5, 5.0)
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

	camera cam = camera::from_parameters(
		vec3d(0.0, 5.5, -30.0), vec3d(0.0, 2.5, 0.0), vec3d(0.0, 1.0, 0.0),
		19.5 * constants::pi / 180.0, 1.0
	);

	return { std::move(result), cam };
}

std::pair<scene, camera> cornell_box_base() {
	auto [result, cam] = red_green_box();

	material matte_white;
	{
		auto &lambert = matte_white.value.emplace<materials::lambertian_reflection>();
		lambert.reflectance.modulation = spectrum::from_rgb(vec3d(0.725, 0.71, 0.68));
	}

	scene::mesh_t plane = create_plane(), box = create_box();

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

	return { std::move(result), cam };
}

std::pair<scene, camera> cornell_box_one_light() {
	auto [result, cam] = cornell_box_base();

	material matte_white;
	{
		auto &lambert = matte_white.value.emplace<materials::lambertian_reflection>();
		lambert.reflectance.modulation = spectrum::from_rgb(vec3d(0.725, 0.71, 0.68));
	}

	scene::mesh_t plane = create_plane();

	entity_info light;
	light.mat = matte_white;
	light.mat.emission.modulation = spectrum::from_rgb(2.0 * vec3d(17.0, 12.0, 4.0));
	result.add_mesh_entity(
		plane,
		transform::scale_rotate_translate(
			vec3d(3.0, 1.0, 3.0), vec3d(0.0, 0.0, 0.0), vec3d(0.0, 7.45, 0.0)
		),
		light
	);

	return { std::move(result), cam };
}

std::pair<scene, camera> cornell_box_two_lights() {
	auto [result, cam] = cornell_box_base();

	material matte_white;
	{
		auto &lambert = matte_white.value.emplace<materials::lambertian_reflection>();
		lambert.reflectance.modulation = spectrum::from_rgb(vec3d(0.725, 0.71, 0.68));
	}

	scene::mesh_t plane = create_plane();

	entity_info light_yellow;
	light_yellow.mat = matte_white;
	light_yellow.mat.emission.modulation = spectrum::from_rgb(vec3d(17.0, 12.0, 4.0));
	result.add_mesh_entity(
		plane,
		transform::scale_rotate_translate(
			vec3d(3.0, 1.0, 3.0), vec3d(0.0, 0.0, 0.0), vec3d(2.0, 7.45, 0.0)
		),
		light_yellow
	);

	entity_info light_blue;
	light_blue.mat = matte_white;
	light_blue.mat.emission.modulation = spectrum::from_rgb(vec3d(4.0, 12.0, 17.0));
	result.add_mesh_entity(
		plane,
		transform::scale_rotate_translate(
			vec3d(3.0, 1.0, 3.0), vec3d(0.0, 0.0, 0.0), vec3d(-2.0, 7.45, 0.0)
		),
		light_blue
	);

	return { std::move(result), cam };
}

std::pair<fluid::renderer::scene, fluid::renderer::camera> glass_ball_box() {
	auto [result, cam] = red_green_box();

	material matte_white;
	{
		auto &lambert = matte_white.value.emplace<materials::lambertian_reflection>();
		lambert.reflectance.modulation = spectrum::from_rgb(vec3d(0.725, 0.71, 0.68));
	}

	material glass;
	{
		auto &specular = glass.value.emplace<materials::specular_transmission>();
		specular.skin.modulation = spectrum::identity;
		specular.index_of_refraction = 1.55;
	}

	scene::mesh_t plane = create_plane();

	entity_info sphere;
	sphere.mat = glass;
	primitives::sphere_primitive prim;
	prim.set_transformation(transform::scale_rotate_translate(
		vec3d(3.0, 3.0, 3.0), vec3d(0.0, 27.5 * constants::pi / 180.0, 0.0), vec3d(0.0, 1.25, 0.0)
	));
	result.add_primitive_entity(prim, sphere);

	entity_info light;
	light.mat = matte_white;
	light.mat.emission.modulation = spectrum::from_rgb(2.0 * vec3d(17.0, 12.0, 4.0));
	result.add_mesh_entity(
		plane,
		transform::scale_rotate_translate(
			vec3d(3.0, 1.0, 3.0), vec3d(0.0, 0.0, 0.0), vec3d(0.0, 7.45, 0.0)
		),
		light
	);

	return { std::move(result), cam };
}


std::pair<scene, camera> fluid_box(vec3d min, vec3d max) {
	scene result;

	material matte_white;
	{
		auto &lambert = matte_white.value.emplace<materials::lambertian_reflection>();
		lambert.reflectance.modulation = spectrum::from_rgb(vec3d(0.725, 0.71, 0.68));
	}

	material matte_red;
	{
		auto &lambert = matte_red.value.emplace<materials::lambertian_reflection>();
		lambert.reflectance.modulation = spectrum::from_rgb(vec3d(0.63, 0.065, 0.05));
	}

	material matte_green;
	{
		auto &lambert = matte_green.value.emplace<materials::lambertian_reflection>();
		lambert.reflectance.modulation = spectrum::from_rgb(vec3d(0.14, 0.45, 0.091));
	}

	scene::mesh_t plane = create_plane();

	vec3d center = 0.5 * (min + max), size = max - min;

	entity_info floor;
	floor.mat = matte_white;
	result.add_mesh_entity(
		plane,
		transform::scale_rotate_translate(
			size, vec3d(constants::pi, 0.0, 0.0), vec3d(center.x, min.y, center.z)
		),
		floor
	);

	entity_info ceiling;
	ceiling.mat = matte_white;
	result.add_mesh_entity(
		plane,
		transform::scale_rotate_translate(
			size, vec3d(-constants::pi, 0.0, 0.0), vec3d(center.x, max.y, center.z)
		),
		ceiling
	);

	entity_info left_wall;
	left_wall.mat = matte_red;
	result.add_mesh_entity(
		plane,
		transform::scale_rotate_translate(
			size, vec3d(0.0, 0.0, 0.5 * constants::pi), vec3d(min.x, center.y, center.z)
		),
		left_wall
	);

	entity_info right_wall;
	right_wall.mat = matte_green;
	result.add_mesh_entity(
		plane,
		transform::scale_rotate_translate(
			size, vec3d(0.0, 0.0, -0.5 * constants::pi), vec3d(max.x, center.y, center.z)
		),
		right_wall
	);

	entity_info back_wall;
	back_wall.mat = matte_white;
	result.add_mesh_entity(
		plane,
		transform::scale_rotate_translate(
			size, vec3d(0.5 * constants::pi, 0.0, 0.0), vec3d(center.x, center.y, max.z)
		),
		back_wall
	);

	// lights
	entity_info light_yellow;
	light_yellow.mat = matte_white;
	light_yellow.mat.emission.modulation = spectrum::from_rgb(vec3d(17.0, 12.0, 4.0));
	result.add_mesh_entity(
		plane,
		transform::scale_rotate_translate(
			vec3d(0.3 * size.x, 1.0, 0.3 * size.z), vec3d(0.0, 0.0, 0.0), vec3d(center.x - 0.25 * size.x, max.y - 0.05, center.z)
		),
		light_yellow
	);

	entity_info light_blue;
	light_blue.mat = matte_white;
	light_blue.mat.emission.modulation = spectrum::from_rgb(vec3d(4.0, 12.0, 17.0));
	result.add_mesh_entity(
		plane,
		transform::scale_rotate_translate(
			vec3d(0.3 * size.x, 1.0, 0.3 * size.z), vec3d(0.0, 0.0, 0.0), vec3d(center.x + 0.25 * size.x, max.y - 0.05, center.z)
		),
		light_blue
	);

	camera cam = camera::from_parameters(
		vec3d(center.x, center.y, min.z - 30.0), center, vec3d(0.0, 1.0, 0.0),
		90.0 * constants::pi / 180.0, 1.0
	);

	return { std::move(result), cam };
}
