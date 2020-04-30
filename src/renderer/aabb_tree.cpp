#include "fluid/renderer/aabb_tree.h"

namespace fluid::renderer {
	void aabb_tree::add_primitive(primitive prim) {
		assert(_node_pool.empty());
		_primitive_pool.emplace_back(std::move(prim));
	}

	void aabb_tree::build() {
		_node_pool.resize(_primitive_pool.size() * 2 - 1);
		// initialize all leaf nodes
		for (std::size_t i = 0; i < _primitive_pool.size(); ++i) {
			primitive &prim = _primitive_pool[i];
			_node_pool[i].bounding_box = prim.get_bounding_box();
			_node_pool[i].prim = &prim;
		}

		// TODO
	}

	double aabb_tree::evaluate_heuristic(aabb3d bbox) {
		vec3d size = bbox.get_size();
		return size.x * size.y + size.x * size.z + size.y * size.z;
	}
}
