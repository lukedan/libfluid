#include "fluid/renderer/primitive.h"

/// \file
/// Implementation of primitives.

#include "fluid/math/constants.h"
#include "fluid/math/warping.h"

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

		surface_sample triangle_primitive::sample_surface(vec2d pos) const {
			surface_sample result;
			if (pos.x > pos.y) {
				pos.x = 1.0 - pos.x;
				result.geometric_normal = geometric_normal;
			} else {
				pos.y = 1.0 - pos.y;
				result.geometric_normal = -geometric_normal;
			}
			result.position = point1 + edge12 * pos.x + edge13 * pos.y;
			result.uv = uv_p1 + uv_e12 * pos.x + uv_e13 * pos.y;
			result.pdf = 1.0 / surface_area;
			return result;
		}

		void triangle_primitive::compute_attributes() {
			vec3d cross = vec_ops::cross(edge12, edge13);
			auto [norm, area] = cross.normalized_length_unchecked();
			geometric_normal = norm;
			surface_area = area;
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
			normal = world_to_local.transposed() * normal;
			return normal.normalized_unchecked();
		}

		vec2d sphere_primitive::get_uv(ray_cast_result r) const {
			vec2d result;
			result.x = ((std::atan2(r.custom[2], r.custom[0]) / constants::pi) + 1.0) * 0.5;
			// for performance
			result.y = (r.custom[1] + 1.0) * 0.5;
			return result;
		}

		surface_sample sphere_primitive::sample_surface(vec2d pos) const {
			vec3d pos_local = warping::unit_sphere_from_unit_square(pos);
			surface_sample result;
			// position, this can be nonuniform after transforming
			result.position = local_to_world * pos_local + local_to_world_offset;
			// compute uv
			result.uv.x = ((std::atan2(pos_local.z, pos_local.x) / constants::pi) + 1.0) * 0.5;
			result.uv.y = (pos_local.y + 1.0) * 0.5; // for performance
			// normal
			result.geometric_normal = world_to_local.transposed() * pos_local;
			// https://math.stackexchange.com/questions/942561/surface-area-of-transformed-sphere
			result.pdf = 1.0;
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

	primitives::surface_sample primitive::sample_surface(vec2d pos) const {
		return std::visit(
			[&](const auto &prim) {
				return prim.sample_surface(pos);
			},
			value
				);
	}
}
