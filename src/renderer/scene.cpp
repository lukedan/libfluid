#include "fluid/renderer/scene.h"

/// \file
/// Implementation of the scene.

namespace fluid::renderer {
	ray intersection_info::spawn_ray(vec3d tangent_dir, double offset) const {
		ray result;
		result.origin = intersection;
		result.origin += geometric_normal * (tangent_dir.y > 0.0 ? offset : -offset);
		result.direction = tangent.transposed() * tangent_dir;
		return result;
	}

	intersection_info intersection_info::from_intersection(const ray &ray, const primitive *prim, ray_cast_result hit) {
		intersection_info result;
		result.uv = prim->get_uv(hit);
		result.surface_bsdf = prim->entity->mat.get_bsdf(result.uv);
		result.geometric_normal = prim->get_geometric_normal(hit);
		result.tangent = compute_arbitrary_tangent_space(result.geometric_normal);
		result.intersection = ray.origin + ray.direction * hit.t;
		return result;
	}


	void scene::add_mesh_entity(const mesh_t &m, const rmat3x4d &trans, entity_info info) {
		entity_info &ent = _entities.emplace_back(std::move(info));
		std::vector<vec3d> trans_pos(m.positions.size());
		for (std::size_t i = 0; i < m.positions.size(); ++i) {
			trans_pos[i] = trans * vec4d(m.positions[i], 1.0);
		}
		for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
			primitive prim;
			prim.entity = &ent;
			auto &tri = prim.value.emplace<primitives::triangle_primitive>();
			std::size_t i1 = m.indices[i], i2 = m.indices[i + 1], i3 = m.indices[i + 2];
			tri.point1 = trans_pos[i1];
			tri.edge12 = trans_pos[i2] - tri.point1;
			tri.edge13 = trans_pos[i3] - tri.point1;
			if (!m.uvs.empty()) {
				tri.uv_p1 = m.uvs[i1];
				tri.uv_e12 = m.uvs[i2] - tri.uv_p1;
				tri.uv_e13 = m.uvs[i3] - tri.uv_p1;
			}
			tri.compute_attributes();
			_tree.add_primitive(std::move(prim));
		}
	}

	void scene::add_primitive_entity(const primitive::union_t &geom, entity_info i) {
		entity_info &ent = _entities.emplace_back(std::move(i));
		primitive prim;
		prim.value = geom;
		prim.entity = &ent;
		_tree.add_primitive(prim);
	}

	void scene::finish() {
		_tree.build();
		// collect light sources
		_lights.clear();
		for (const primitive &prim : _tree.get_primitives()) {
			if (!prim.entity->mat.emission.modulation.near_zero()) {
				_lights.emplace_back(&prim);
			}
		}
	}

	std::tuple<const primitive*, ray_cast_result, intersection_info> scene::ray_cast(const ray &r) const {
		auto [prim, res] = _tree.ray_cast(r);
		if (prim) {
			return { prim, res, intersection_info::from_intersection(r, prim, res) };
		}
		return { nullptr, ray_cast_result(), intersection_info() };
	}

	bool scene::test_visibility(vec3d p1, vec3d p2, double eps) const {
		vec3d diff = p2 - p1;
		vec3d offset = diff.normalized_unchecked() * eps;
		ray r;
		r.direction = diff - 2.0 * offset;
		r.origin = p1 + offset;
		auto [prim, res] = _tree.ray_cast(r, 1.0);
		return prim == nullptr;
	}

	ray scene::spawn_ray_from(vec3d pos, vec3d dir, vec3d norm, double offset) {
		ray result;
		result.origin = pos + norm * offset;
		result.direction = compute_arbitrary_tangent_space(norm).transposed() * dir;
		return result;
	}
}
