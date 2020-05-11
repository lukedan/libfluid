#include "fluid/renderer/aabb_tree.h"

/// \file
/// Implementation of the AABB tree.

#include <stack>
#include <iostream>

#include "fluid/data_structures/short_vec.h"

/*#define FLUID_RENDERER_PROFILE_AABB_TREE*/

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

	/// Stores information about a step in building the AABB tree.
	struct _build_step {
		/// Default constructor.
		_build_step() = default;
		/// Initializes all fields of this struct.
		_build_step(aabb_tree::node *&parent, std::size_t b, std::size_t e) : parent_ptr(&parent), begin(b), end(e) {
		}

		/// When this subtree is built, this pointer should be set to point to the root of this subtree.
		aabb_tree::node **parent_ptr = nullptr;
		std::size_t
			begin = 0, ///< Index of the first leaf node for this step.
			end = 0; ///< Index past the last leaf node for this step.

		/// Returns the number of primitives that requires consideration in this step.
		[[nodiscard]] std::size_t primitive_count() const {
			return end - begin;
		}
	};
	/// Stores an reference to a leaf node and a centroid position.
	struct _leaf_ref {
		/// Default constructor.
		_leaf_ref() = default;
		/// Initializes this reference from the given leaf.
		explicit _leaf_ref(aabb_tree::node &n) : centroid(n.bounding_box.get_center()), node(&n) {
		}

		vec3d centroid; ///< The centroid of the node.
		aabb_tree::node *node = nullptr; ///< The leaf.
	};
	/// Stores information about a bucket.
	struct _bucket {
		constexpr static double
			double_min = std::numeric_limits<double>::min(), ///< The minimum value of a \p double.
			double_max = std::numeric_limits<double>::max(); ///< The maximum value of a \p double.

		/// Initializes \ref aabb_bound to be the maximum negative box.
		_bucket() : aabb_bound(
			vec3d(double_max, double_max, double_max),
			vec3d(double_min, double_min, double_min)
		) {
		}

		/// Returns \ref count times the heuristic of \ref aabb_bound.
		[[nodiscard]] double heuristic_term() const {
			return static_cast<double>(count) * aabb_tree::evaluate_heuristic(aabb_bound);
		}

		std::size_t count = 0; ///< The number of primitives in this bucket.
		aab3d aabb_bound; ///< The bound of all AABBs in this bucket.
	};
	void aabb_tree::build() {
		constexpr std::size_t num_buckets = 12;

		if (_primitive_pool.empty()) {
			return;
		}
		_node_pool.resize(_primitive_pool.size() * 2 - 1);
		std::vector<_leaf_ref> leaves(_primitive_pool.size());
		// create leaf nodes
		for (std::size_t i = 0; i < _primitive_pool.size(); ++i) {
			const primitive &prim = _primitive_pool[i];
			_node_pool[i].prim = &prim;
			_node_pool[i].bounding_box = prim.get_bounding_box();
			leaves[i] = _leaf_ref(_node_pool[i]);
		}
		// start building
		std::stack<_build_step> stk;
		stk.emplace(_root, 0, _primitive_pool.size());
		std::size_t alloc = _primitive_pool.size(); // next node that can be allocated
		while (!stk.empty()) {
			_build_step step = stk.top();
			stk.pop();
			// first handle special cases
			switch (step.primitive_count()) {
			case 1:
				*step.parent_ptr = leaves[step.begin].node;
				continue;
			case 2:
				{
					node &n = _node_pool[alloc++];
					n.child1 = leaves[step.begin].node;
					n.child2 = leaves[step.begin + 1].node;
					n.bounding_box = aab3d::bounding(n.child1->bounding_box, n.child2->bounding_box);
					*step.parent_ptr = &n;
				}
				continue;
			}
			// general case
			// compute centroid bounds
			aab3d
				centroid_bound = aab3d::containing(leaves[step.begin].centroid),
				aabb_bound = leaves[step.begin].node->bounding_box;
			for (std::size_t i = step.begin + 1; i < step.end; ++i) {
				centroid_bound.make_contain(leaves[i].centroid);
				aabb_bound = aab3d::bounding(aabb_bound, leaves[i].node->bounding_box);
			}
			vec3d size = centroid_bound.get_size();
			std::size_t sep_dim = size.y > size.x ? 1 : 0;
			if (size.z > size[sep_dim]) {
				sep_dim = 2;
			}
			// put all nodes into buckets
			_bucket buckets[num_buckets];
			double bucket_range = size[sep_dim] / static_cast<double>(num_buckets);
			for (std::size_t i = step.begin; i < step.end; ++i) {
				const _leaf_ref &leaf = leaves[i];
				std::size_t buck = static_cast<std::size_t>(
					(leaf.centroid[sep_dim] - centroid_bound.min[sep_dim]) / bucket_range
				);
				buck = std::min(buck, num_buckets - 1);
				++buckets[buck].count;
				buckets[buck].aabb_bound = aab3d::bounding(buckets[buck].aabb_bound, leaf.node->bounding_box);
			}
			// find optimal bucket
			_bucket sep_bound_max_cache[num_buckets - 1];
			{ // find the bounding box of buckets [1 ... n - 1, n]
				_bucket cur = buckets[num_buckets - 1];
				for (std::size_t i = num_buckets - 1; i > 0; ) {
					--i;
					sep_bound_max_cache[i] = cur;
					cur.aabb_bound = aab3d::bounding(cur.aabb_bound, buckets[i].aabb_bound);
					cur.count += buckets[i].count;
				}
			}
			std::size_t min_heuristic_split = 0;
			{
				double min_heuristic = std::numeric_limits<double>::max();
				_bucket sep_bound_min = buckets[0];
				for (std::size_t split = 0; split < num_buckets - 1; ++split) {
					_bucket sep_bound_max = sep_bound_max_cache[split];
					double heuristic =
						0.125 +
						(sep_bound_min.heuristic_term() + sep_bound_max.heuristic_term()) /
						evaluate_heuristic(aabb_bound);
					if (heuristic < min_heuristic) {
						min_heuristic = heuristic;
						min_heuristic_split = split;
					}
					// update sep_bound_min
					sep_bound_min.aabb_bound =
						aab3d::bounding(sep_bound_min.aabb_bound, buckets[split + 1].aabb_bound);
					sep_bound_min.count += buckets[split + 1].count;
				}
			}
			// split
			double split_pos =
				centroid_bound.min[sep_dim] + static_cast<double>(min_heuristic_split + 1) * bucket_range;
			std::size_t end_before = step.begin;
			for (std::size_t i = step.begin; i < step.end; ++i) {
				if (leaves[i].centroid[sep_dim] < split_pos) {
					std::swap(leaves[end_before], leaves[i]);
					++end_before;
				}
			}
			node &n = _node_pool[alloc++];
			n.bounding_box = aabb_bound;
			*step.parent_ptr = &n;
			stk.emplace(n.child1, step.begin, end_before);
			stk.emplace(n.child2, end_before, step.end);
		}
	}

	/// Tests if the intersection is valid, i.e., if it's not nan and if the value is less than the max value.
	bool _intersects(double t, double max_t) {
		return std::isless(t, max_t);
	}
	std::pair<const primitive*, ray_cast_result> aabb_tree::ray_cast(const ray &r, double max_t) const {
#ifdef FLUID_RENDERER_PROFILE_AABB_TREE
		std::size_t aab_tests = 1, prim_tests = 0;
#endif
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
#ifdef FLUID_RENDERER_PROFILE_AABB_TREE
				++prim_tests;
#endif
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
#ifdef FLUID_RENDERER_PROFILE_AABB_TREE
				aab_tests += 2;
#endif
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
#ifdef FLUID_RENDERER_PROFILE_AABB_TREE
		std::cout << "aab " << aab_tests << "  prim " << prim_tests << "\n";
#endif
		return { hit, hit_res };
	}

	double aabb_tree::evaluate_heuristic(aab3d bbox) {
		vec3d size = bbox.get_size();
		return size.x * size.y + size.x * size.z + size.y * size.z;
	}
}
