#pragma once

/// \file
/// Definition of the AABB tree.

#include <vector>

#include <emmintrin.h>

#include "common.h"
#include "primitive.h"

namespace fluid::renderer {
	/// A bounding volume hierarchy that uses axis aligned bounding boxes.
	class aabb_tree {
	public:
		/// A node in the tree.
		struct node {
			/// Default constructor.
			node() {
			}
			/// Destructor.
			~node();

			union {
				/// Children bounding boxes. The lower 64 bits stores the values for \ref child1.
				aab3<__m128d> children_bb;
				aab3d leaf_bb; ///< Bounding box of this node, only valid for leaf nodes.
			};
			/// The first child. This is \p nullptr if this node is a leaf node, and non-null otherwise. Therefore
			/// this member determines which member of the next union is active.
			node *child1 = nullptr;
			union {
				node *child2; ///< The second child.
				const primitive *prim; ///< The primitive associated with a leaf node.
			};

			/// Returns whether this node is a leaf node.
			bool is_leaf() const {
				return child1 == nullptr;
			}

			/// Sets the children bounding boxes of this node.
			void set_children_bounding_boxes(aab3d c1, aab3d c2);
			/// Sets the children bounding boxes of this node, assuming that the child nodes are leaves (i.e., that
			/// \ref leaf_bb is active for the children).
			void set_children_bounding_boxes_leaf();
		};

		/// Default constructor.
		aabb_tree() = default;
		/// No copy construction.
		aabb_tree(const aabb_tree&) = delete;
		/// Move constructor.
		aabb_tree(aabb_tree&&) noexcept;
		/// No copy assignment.
		aabb_tree &operator=(const aabb_tree&) = delete;
		/// Move assignment.
		aabb_tree &operator=(aabb_tree&&) noexcept;

		/// Adds a primitive to the tree. This function clears \ref _node_pool and \ref _root.
		void add_primitive(primitive);

		/// Builds the tree. If \ref add_primitive() is called after this, then this function needs to be called
		/// again before \ref ray_cast() is called.
		void build();

		/// Performs ray casting. The additional parameter is used to limit the range of the ray cast.
		std::pair<const primitive*, ray_cast_result> ray_cast(
			const ray&, double max_t = std::numeric_limits<double>::max()
		) const;

		/// Returns the list of all primitives.
		const std::vector<primitive> &get_primitives() const {
			return _primitive_pool;
		}

		/// Evaluates the given bounding box to decide whether or not to merge two subtrees. The smaller the
		/// heuristic is, the better. The default heuristic is based on the surface area of the bounding box.
		[[nodiscard]] static double evaluate_heuristic(aab3d);
	private:
		std::vector<node> _node_pool; ///< Storage for all nodes.
		std::vector<primitive> _primitive_pool; ///< Storage for all primitives.
		node *_root = nullptr; ///< The root node.
	};
}
