#pragma once

/// \file
/// Declaration of primitives.

#include <variant>

#include "fluid/math/vec.h"
#include "fluid/renderer/common.h"

namespace fluid::renderer {
	/// A triangle primitive.
	struct triangle_primitive {
		vec3d points[3]; ///< Three vertices of this triangle.

		/// Returns the bounding box.
		[[nodiscard]] aabb3d get_bounding_box() const {
			return aabb3d::containing(points[0], points[1], points[2]);
		}
	};
	/// A sphere primitive.
	struct sphere_primitive {
		[[nodiscard]] aabb3d get_bounding_box() const {

		}
	};

	/// A generic primitive.
	struct primitive {
		/// The union used store the primitive.
		using union_t = std::variant<triangle_primitive, sphere_primitive>;

		/// Returns the bounding box of this primitive.
		[[nodiscard]] aabb3d get_bounding_box() const {
			return std::visit(
				[](const auto &prim) {
					return prim.get_bounding_box();
				},
				value
					);
		}

		union_t value; ///< The value of this primitive.
	};
}