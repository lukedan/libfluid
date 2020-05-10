#pragma once

/// \file
/// Definition of the scene.

#include <deque>

#include "../math/mat.h"
#include "../data_structures/mesh.h"
#include "material.h"
#include "aabb_tree.h"

namespace fluid::renderer {
	/// Information about an entity.
	struct entity_info {
		material mat; ///< The material of this entity.
	};

	/// Stores information about an intersection.
	struct intersection_info {
		bsdf surface_bsdf; ///< The BSDF function of this surface.
		rmat3d tangent; ///< The matrix used to convert directions from world space into tangent space.
		vec3d
			intersection, ///< The exact point of intersection.
			geometric_normal; ///< The geometric normal of the intersection point.
		vec2d uv; ///< The UV at the intersection point.

		/// Spawns a new ray given its direction in tangent space. The resulting ray's direction has the same length
		/// as the input direction.
		ray spawn_ray(vec3d tangent_dir, double offset = 1e-6) const;

		/// Creates a \ref intersection_info from the given raycast result.
		static intersection_info from_intersection(const ray&, const primitive*, ray_cast_result);
	};

	/// Manages scene entities.
	class scene {
	public:
		using mesh_t = mesh<double, std::size_t, double, double, vec3d>; ///< Mesh type.

		/// Adds a transformed mesh entity to the scene.
		void add_mesh_entity(const mesh_t&, const rmat3x4d&, entity_info);
		/// Adds a primitive to the scene.
		void add_primitive_entity(const primitive::union_t&, entity_info);
		/// Finishes building the scene.
		void finish();

		/// Returns the list of all primitives that emit light.
		const std::vector<const primitive*> &get_lights() const {
			return _lights;
		}

		/// Performs ray casting.
		std::tuple<const primitive*, ray_cast_result, intersection_info> ray_cast(const ray&) const;
		/// Tests the visibility between two points.
		bool test_visibility(vec3d, vec3d, double eps = 1e-6) const;

		/// Spawns a ray given the position, direction in tangent space, and normal. It is assumed that the direction
		/// is in the same hemisphere as the normal.
		static ray spawn_ray_from(vec3d pos, vec3d tangent_dir, vec3d normal, double offset = 1e-6);
	private:
		aabb_tree _tree; ///< The AABB tree.
		std::deque<entity_info> _entities; ///< Information about all entities.
		std::vector<const primitive*> _lights; ///< The list of light sources.
	};
}
