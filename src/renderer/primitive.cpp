#include "fluid/renderer/primitive.h"

/// \file
/// Implementation of primitives.

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
