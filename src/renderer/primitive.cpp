#include "fluid/renderer/primitive.h"

/// \file
/// Implementation of primitives.

#include "fluid/math/constants.h"

namespace fluid::renderer {
	namespace primitives {
		aab3d triangle_primitive::get_bounding_box() const {
			return aab3d::containing(point1, point1 + edge12, point1 + edge13);
		}

		ray_cast_result triangle_primitive::ray_cast(const ray &r) const {
			ray_cast_result result;
			vec3d hit = ray_triangle_intersection_edges(r.origin, r.direction, point1, edge12, edge13);
			result.t = hit.x;
			result.custom[0] = hit.y;
			result.custom[1] = hit.z;
			return result;
		}

		vec3d triangle_primitive::get_geometric_normal(ray_cast_result) const {
			return geometric_normal;
		}

		vec2d triangle_primitive::get_uv(ray_cast_result hit) const {
			return uv_p1 + hit.custom[0] * uv_e12 + hit.custom[1] * uv_e13;
		}

		void triangle_primitive::compute_geometric_normal() {
			geometric_normal = vec_ops::cross(edge12, edge13).normalized_unchecked();
		}


		aab3d sphere_primitive::get_bounding_box() const {
			// https://tavianator.com/exact-bounding-boxes-for-spheres-ellipsoids/
			vec3d
				r1 = local_to_world.row(0),
				r2 = local_to_world.row(1),
				r3 = local_to_world.row(2);
			vec3d half_extent(r1.length(), r2.length(), r3.length());
			return aab3d(local_to_world_offset - half_extent, local_to_world_offset + half_extent);
		}

		ray_cast_result sphere_primitive::ray_cast(const ray &r) const {
			ray_cast_result result;
			vec3d
				origin = world_to_local * r.origin + world_to_local_offset,
				direction = world_to_local * r.direction;
			vec2d hit = unit_radius_sphere_ray_intersection(origin, direction);
			if (std::isnan(hit.x)) { // no intersection
				result.t = hit.x;
				return result;
			}
			result.t = hit.x > 0.0 ? hit.x : hit.y;
			vec3d hit_point = origin + result.t * direction;
			result.custom[0] = hit_point.x;
			result.custom[1] = hit_point.y;
			result.custom[2] = hit_point.z;
			return result;
		}

		vec3d sphere_primitive::get_geometric_normal(ray_cast_result r) const {
			vec3d normal(r.custom[0], r.custom[1], r.custom[2]);
			normal = local_to_world * normal;
			return normal.normalized_unchecked();
		}

		vec2d sphere_primitive::get_uv(ray_cast_result r) const {
			vec2d result;
			result.x = ((std::atan2(r.custom[2], r.custom[0]) / constants::pi) + 1.0) * 0.5;
			result.y = (r.custom[1] + 1.0) * 0.5;
			return result;
		}

		void sphere_primitive::set_transformation(rmat3x4d trans) {
			local_to_world = rmat3d::from_rows(
				vec_ops::slice<0, 3>(trans.row(0)),
				vec_ops::slice<0, 3>(trans.row(1)),
				vec_ops::slice<0, 3>(trans.row(2))
			);
			local_to_world_offset = trans.column(3);
			rmat4d full = rmat4d::from_rows(trans.row(0), trans.row(1), trans.row(2), vec4d(0.0, 0.0, 0.0, 1.0));
			full = full.get_inverse();
			world_to_local = rmat3d::from_rows(
				vec_ops::slice<0, 3>(full.row(0)),
				vec_ops::slice<0, 3>(full.row(1)),
				vec_ops::slice<0, 3>(full.row(2))
			);
			world_to_local_offset = vec_ops::slice<0, 3>(full.column(3));
		}
	}


	aab3d primitive::get_bounding_box() const {
		return std::visit(
			[](const auto &prim) {
				return prim.get_bounding_box();
			},
			value
				);
	}

	ray_cast_result primitive::ray_cast(const ray &r) const {
		return std::visit(
			[&](const auto &prim) {
				return prim.ray_cast(r);
			},
			value
				);
	}

	vec3d primitive::get_geometric_normal(ray_cast_result hit) const {
		return std::visit(
			[&](const auto &prim) {
				return prim.get_geometric_normal(hit);
			},
			value
				);
	}

	vec2d primitive::get_uv(ray_cast_result hit) const {
		return std::visit(
			[&](const auto &prim) {
				return prim.get_uv(hit);
			},
			value
				);
	}
}
