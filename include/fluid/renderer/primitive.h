#pragma once

/// \file
/// Declaration of primitives.

#include <variant>

#include "fluid/math/vec.h"
#include "fluid/math/mat.h"
#include "fluid/math/intersection.h"
#include "fluid/data_structures/aab.h"
#include "fluid/renderer/common.h"

namespace fluid::renderer {
	struct entity_info;

	/// Stores the resutls of a ray cast operation.
	struct ray_cast_result {
		double t = 0.0; ///< The \p t value of the ray. If the ray cast didn't hit, then this value will be \p nan.
		double custom[3]{}; ///< Custom data used by each primitive type.

		/// Returns whether this ray cast is a hit.
		bool is_hit() const {
			return !std::isnan(t);
		}
	};

	/// Definition of different types of primitives.
	namespace primitives {
		/// Stores the result of sampling the surface of a primitive.
		struct surface_sample {
			vec3d
				position, ///< Position on the surface in world coordinates.
				geometric_normal; ///< The geometric normal.
			vec2d uv; ///< The UV at the sampled position.
			double pdf = 0.0; ///< The probability density function of this sample's position.
		};

		/// A triangle primitive.
		struct triangle_primitive {
			vec3d
				point1, ///< The first vertex of this triangle.
				edge12, ///< Edge from the first vertex to the second vertex.
				edge13, ///< Edge from the first vertex to the third vertex.
				geometric_normal; ///< The geometric normal of this triangle.
			vec2d
				uv_p1, ///< The UV of \ref point1.
				uv_e12, ///< The UV difference of \ref edge12.
				uv_e13; ///< The UV difference of \ref edge13.
			double surface_area = 0.0; ///< The surface area of this primitive. This includes both sides.

			/// Returns the bounding box.
			[[nodiscard]] aab3d get_bounding_box() const;
			/// Returns the result of \ref ray_triangle_intersection_edges().
			[[nodiscard]] ray_cast_result ray_cast(const ray&) const;
			/// Returns \ref geometric_normal.
			[[nodiscard]] vec3d get_geometric_normal(ray_cast_result) const;
			/// Returns the UV at the given intersection.
			[[nodiscard]] vec2d get_uv(ray_cast_result) const;

			/// Samples the surface of this triangle.
			[[nodiscard]] surface_sample sample_surface(vec2d) const;

			/// Computes the normal and surface area of this triangle.
			void compute_attributes();
		};
		/// A sphere primitive. The primitive is a sphere at the origin with radius 1 transformed by the given
		/// transformation matrix.
		struct sphere_primitive {
			rmat3d
				world_to_local, ///< Transforms directions from world coordinates to local coordinates.
				local_to_world; ///< Transforms directions from world coordinates to local coordinates.
			vec3d
				world_to_local_offset, ///< The offset used when transforming from world to local coordinates.
				local_to_world_offset; ///< The offset used when transforming from local to world coordinates.

			/// Returns the bounding box.
			[[nodiscard]] aab3d get_bounding_box() const;
			/// Returns the ray cast result.
			[[nodiscard]] ray_cast_result ray_cast(const ray&) const;
			/// Computes the normal at the given intersection.
			[[nodiscard]] vec3d get_geometric_normal(ray_cast_result) const;
			/// Computes the UV at the given intersection.
			[[nodiscard]] vec2d get_uv(ray_cast_result) const;

			/// Samples the surface of this sphere. This function is generally inaccurate and should be avoided.
			[[nodiscard]] surface_sample sample_surface(vec2d) const;

			/// Sets the transformation of this sphere.
			void set_transformation(rmat3x4d);
		};
	}

	/// A generic primitive.
	struct primitive {
		/// The union used store the primitive.
		using union_t = std::variant<
			primitives::triangle_primitive,
			primitives::sphere_primitive
		>;

		/// Forwards the call to underlying primitive types.
		[[nodiscard]] aab3d get_bounding_box() const;
		/// Performs ray casting.
		[[nodiscard]] ray_cast_result ray_cast(const ray&) const;
		/// Returns an orthonormal matrix that converts from world space directions to tangent space directions.
		[[nodiscard]] vec3d get_geometric_normal(ray_cast_result) const;
		/// Returns the UV at the given intersection.
		[[nodiscard]] vec2d get_uv(ray_cast_result) const;

		/// Samples the surface of this primitive.
		///
		/// \return A point on the surface and the surface normal at that point.
		[[nodiscard]] primitives::surface_sample sample_surface(vec2d) const;

		union_t value; ///< The value of this primitive.
		entity_info *entity = nullptr; ///< The entity associated with this primitive.
	};
}