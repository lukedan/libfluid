#pragma once

/// \file
/// Mesh related data structures.

#include <vector>

#include "../math/vec.h"

namespace fluid {
	/// A mesh.
	template <typename PosT, typename ColorT, typename NormalT, typename IndexT> struct mesh {
		std::vector<vec3<PosT>> positions; ///< Vertex positions.
		std::vector<vec3<NormalT>> normals; ///< Vertex normals.
		std::vector<IndexT> indices; ///< Triangle indices.
		std::vector<ColorT> colors; ///< Vertex colors.

		/// Clears all data in this mesh.
		void clear() {
			positions.clear();
			normals.clear();
			indices.clear();
			colors.clear();
		}

		/// Saves this mesh to an .OBJ file.
		void save_obj(std::ostream &out) const {
			for (auto &p : positions) {
				out << "v " << p.x << " " << p.y << " " << p.z << "\n";
			}
			if (!normals.empty()) {
				for (auto &n : normals) {
					out << "vn " << n.x << " " << n.y << " " << n.z << "\n";
				}
				for (std::size_t i = 0; i < indices.size() - 2; ) {
					out << "f";
					for (std::size_t j = 0; j < 3; ++i, ++j) {
						out << " " << indices[i] << "//" << indices[i];
					}
					out << "\n";
				}
			} else {
				for (std::size_t i = 0; i < indices.size() - 2; i += 3) {
					out << "f " << indices[i] + 1 << " " << indices[i + 1] + 1 << " " << indices[i + 2] + 1 << "\n";
				}
			}
		}
	};
}
