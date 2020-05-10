#include "fluid/renderer/aabb_tree.h"

/// \file
/// Implementation of the AABB tree.

#include <stack>

#include "fluid/data_structures/short_vec.h"

namespace fluid::renderer {
	aabb_tree::aabb_tree(aabb_tree &&src) noexcept :
		_node_pool(std::move(src._node_pool)),
		_primitive_pool(std::move(src._primitive_pool)),
		_root(src._root) {
	}

	aabb_tree &aabb_tree::operator=(aabb_tree &&src) noexcept {
		_node_pool = std::move(src._node_pool);
		_primitive_pool = std::move(src._primitive_pool);
		_root = src._root;
		return *this;
	}

	void aabb_tree::add_primitive(primitive prim) {
		_node_pool.clear();
		_root = nullptr;
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
		// actually build the tree
		_root = &_node_pool[0];
		node *alloc = &_node_pool[_primitive_pool.size()];
		for (std::size_t i = 1; i < _primitive_pool.size(); ++i) {
			node *target = &_node_pool[i]; // new allocated node
			// insert node into the tree
			node **handle = &_root;
			aab3d bounding = aab3d::bounding(_root->bounding_box, target->bounding_box);
			double
				target_heuristic = evaluate_heuristic(target->bounding_box),
				subtree_heuristic = evaluate_heuristic(_root->bounding_box);
			while (!(*handle)->is_leaf()) {
				(*handle)->bounding_box = bounding;
				aab3d
					b1 = aab3d::bounding(target->bounding_box, (*handle)->child1->bounding_box),
					b2 = aab3d::bounding(target->bounding_box, (*handle)->child2->bounding_box);
				double
					old_heuristic1 = evaluate_heuristic((*handle)->child1->bounding_box),
					old_heuristic2 = evaluate_heuristic((*handle)->child2->bounding_box),
					new_heuristic1 = evaluate_heuristic(b1), new_heuristic2 = evaluate_heuristic(b2);
				double
					merge_heu1 = new_heuristic1 + old_heuristic2,
					merge_heu2 = old_heuristic1 + new_heuristic1,
					split_heu = subtree_heuristic + target_heuristic;
				if (split_heu < merge_heu1 && split_heu < merge_heu2) {
					// early termination: split the current node
					break;
				} else {
					// keep going down
					if (merge_heu1 < merge_heu2) {
						handle = &(*handle)->child1;
						bounding = b1;
						subtree_heuristic = new_heuristic1;
					} else {
						handle = &(*handle)->child2;
						bounding = b2;
						subtree_heuristic = new_heuristic2;
					}
				}
			}
			alloc->bounding_box = bounding;
			alloc->child1 = (*handle);
			alloc->child2 = target;
			*handle = alloc;
			++alloc;
		}
	}

	/// Tests if the intersection is valid, i.e., if it's not nan and if the value is less than the max value.
	bool _intersects(double t, double max_t) {
		return std::isless(t, max_t);
	}
	std::pair<const primitive*, ray_cast_result> aabb_tree::ray_cast(const ray &r, double max_t) const {
		if (!_intersects(
			aab_ray_intersection(_root->bounding_box.min, _root->bounding_box.max, r.origin, r.direction).x, max_t
		)) {
			return { nullptr, ray_cast_result() };
		}
		std::stack<node*, short_vec<node*, 64>> nodes;
		nodes.emplace(_root);
		const primitive *hit = nullptr;
		ray_cast_result hit_res;
		hit_res.t = max_t;
		while (!nodes.empty()) {
			node *current = nodes.top();
			nodes.pop();
			if (current->is_leaf()) {
				ray_cast_result result = current->prim->ray_cast(r);
				if (_intersects(result.t, hit_res.t)) {
					hit = current->prim;
					hit_res = result;
				}
			} else {
				node *c1 = current->child1, *c2 = current->child2;
				double
					i1 = aab_ray_intersection(c1->bounding_box.min, c1->bounding_box.max, r.origin, r.direction).x,
					i2 = aab_ray_intersection(c2->bounding_box.min, c2->bounding_box.max, r.origin, r.direction).x;
				bool hit1 = _intersects(i1, hit_res.t), hit2 = _intersects(i2, hit_res.t);
				if (!hit1) {
					if (hit2) {
						nodes.emplace(current->child2);
					}
				} else if (!hit2) {
					nodes.emplace(current->child1);
				} else { // both are not nan
					if (i1 < i2) { // check child1 first
						nodes.emplace(current->child2);
						nodes.emplace(current->child1);
					} else {
						nodes.emplace(current->child1);
						nodes.emplace(current->child2);
					}
				}
			}
		}
		return { hit, hit_res };
	}

	double aabb_tree::evaluate_heuristic(aab3d bbox) {
		vec3d size = bbox.get_size();
		return size.x * size.y + size.x * size.z + size.y * size.z;
	}
}
