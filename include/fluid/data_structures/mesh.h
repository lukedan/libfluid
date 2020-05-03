#pragma once

/// \file
/// Mesh related data structures.

#include <vector>

#include "../math/vec.h"

namespace fluid {
	/// A mesh.
	template <typename PosT, typename IndexT, typename NormalT, typename UvT, typename ColorT> struct mesh {
		std::vector<vec3<PosT>> positions; ///< Vertex positions.
		std::vector<vec3<NormalT>> normals; ///< Vertex normals.
		std::vector<IndexT> indices; ///< Triangle indices.
		std::vector<ColorT> colors; ///< Vertex colors.
		std::vector<vec2<UvT>> uvs; ///< The UVs.

		/// Clears all data in this mesh.
		void clear() {
			positions.clear();
			normals.clear();
			indices.clear();
			colors.clear();
			uvs.clear();
		}

		/// Generates normals for all vertices.
		void generate_normals() {
			if (!normals.empty()) {
				std::fill(normals.begin(), normals.end(), vec3<NormalT>());
			}
			normals.resize(positions.size());
			for (std::size_t i = 0; i + 2 < indices.size(); i += 3) {
				IndexT i1 = indices[i], i2 = indices[i + 1], i3 = indices[i + 2];
				vec3<NormalT> norm(vec_ops::cross(positions[i2] - positions[i1], positions[i3] - positions[i1]));
				normals[i1] += norm;
				normals[i2] += norm;
				normals[i3] += norm;
			}
			for (vec3<NormalT> &norm : normals) {
				norm = norm.normalized_checked().value_or(vec3<NormalT>(1, 0, 0));
			}
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
			}
			if (!uvs.empty()) {
				for (auto &uv : uvs) {
					out << "vt " << uv.x << " " << uv.y << "\n";
				}
			}

			// faces
			if (normals.empty() && uvs.empty()) {
				for (std::size_t i = 0; i < indices.size() - 2; i += 3) {
					out << "f " << indices[i] + 1 << " " << indices[i + 1] + 1 << " " << indices[i + 2] + 1 << "\n";
				}
			} else if (normals.empty()) {
				for (std::size_t i = 0; i < indices.size() - 2; ) {
					out << "f";
					for (std::size_t j = 0; j < 3; ++i, ++j) {
						IndexT id = indices[i] + 1;
						out << " " << id << "/" << id;
					}
					out << "\n";
				}
			} else {
				for (std::size_t i = 0; i < indices.size() - 2; ) {
					out << "f";
					for (std::size_t j = 0; j < 3; ++i, ++j) {
						IndexT id = indices[i] + 1;
						out << " " << id << "/";
						if (!uvs.empty()) {
							out << id;
						}
						out << "/" << id;
					}
					out << "\n";
				}
			}
		}
	};
}
