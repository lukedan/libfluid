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
			// TODO also save normals
			for (auto &p : positions) {
				out << "v " << p.x << " " << p.y << " " << p.z << "\n";
			}
			for (std::size_t i = 0; i < indices.size() - 2; i += 3) {
				out << "f " << indices[i] + 1 << " " << indices[i + 1] + 1 << " " << indices[i + 2] + 1 << "\n";
			}
		}
	};
}
