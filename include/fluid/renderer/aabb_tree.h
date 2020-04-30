#pragma once

/// \file
/// Definition of the AABB tree.

#include <vector>

#include "common.h"
#include "primitive.h"

namespace fluid::renderer {
	/// A bounding volume hierarchy that uses axis aligned bounding boxes.
	class aabb_tree {
	public:
		/// A node in the tree.
		struct node {
			aabb3d bounding_box; ///< The bounding box of this node.
			/// The first child. This is \p nullptr if this node is a leaf node, and non-null otherwise. Therefore
			/// this member determines which member of the next union is active.
			node *child1 = nullptr;
			union {
				node *child2; ///< The second child.
				const primitive *prim; ///< The primitive associated with a leaf node.
			};
		};

		/// Adds a primitive to the tree. This must be done before the tree is built.
		void add_primitive(primitive);

		/// Builds the tree. \ref add_primitive() must not be called after this.
		void build();

		/// Evaluates the given bounding box to decide whether or not to merge two subtrees. The smaller the
		/// heuristic is, the better. The default heuristic is based on the surface area of the bounding box.
		[[nodiscard]] static double evaluate_heuristic(aabb3d);
	private:
		std::vector<node> _node_pool; ///< Storage for all nodes.
		std::vector<primitive> _primitive_pool; ///< Storage for all primitives.
		node *root = nullptr; ///< The root node.
	};
}
