#pragma once

/// \file
/// Declaration of point cloud related functions.

#include <vector>
#include <iostream>

#include "../math/vec.h"

namespace fluid {
	/// Point cloud related functions.
	namespace point_cloud {
		/// Saves a point cloud to the given output stream.
		template <typename It> inline void save_to_naive(std::ostream &out, It begin, It end) {
			for (auto c = begin; c != end; ++c) {
				out << c->x << " " << c->y << " " << c->z << "\n";
			}
		}
		/// Loads a point cloud from the given input stream. This function keeps reading until reading fails or the
		/// desired number of points is reached.
		template <typename Callback> inline void load_from_naive(
			std::istream &in, Callback &cb, std::size_t count = std::numeric_limits<std::size_t>::max()
		) {
			for (std::size_t i = 0; i < count; ++i) {
				vec3d v;
				if (in >> v.x >> v.y >> v.z) {
					cb(v);
				} else {
					break;
				}
			}
		}
		/// \overload
		[[nodiscard]] std::vector<vec3d> load_from_naive(
			std::istream&, std::size_t = std::numeric_limits<std::size_t>::max()
		);
	}
}
