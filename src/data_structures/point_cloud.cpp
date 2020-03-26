#include "data_structures/point_cloud.h"

/// \file
/// Implementation of certain point cloud related functions.

namespace fluid {
	namespace point_cloud {
		std::vector<vec3d> load_from_naive(std::istream &in, std::size_t count) {
			std::vector<vec3d> result;
			load_from_naive(
				in,
				[&result](vec3d v) {
					result.emplace_back(v);
				},
				count
					);
			return result;
		}
	}
}
