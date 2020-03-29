#pragma once

/// \file
/// Definition of the space hashing data structure.

#include <vector>
#include <algorithm>

#include "grid.h"
#include "particle.h"

namespace fluid {
	/// A space-hashing data structure. Each grid stores particles in that grid.
	template <std::size_t Dim, typename T> struct space_hashing {
	public:
		using size_type = vec<Dim, std::size_t>; ///< Used to store sizes and indices.

		/// Default constructor.
		space_hashing() = default;
		/// Initializes the table to the given size.
		explicit space_hashing(size_type size) : _table(size) {
		}

		/// Resizes the grid, clearing all data in the process.
		void resize(size_type size) {
			_table = _grid_type(size);
			_chain.clear();
			_occupied_cells.clear();
		}

		/// Adds an object to the grid. The object must not be moved in memory after it has been added until
		/// \ref clear() has been called. The object will not be added if it falls out of the grid.
		///
		/// \return Whether the object is in the grid and has been added.
		bool add_object_at(size_type index, T obj) {
			bool in_bounds = true;
			vec_ops::for_each(
				[&in_bounds](std::size_t coord, std::size_t max) {
					if (coord >= max) {
						in_bounds = false;
					}
				},
				index, _table.get_size()
					);
			if (in_bounds) {
				add_object_at_unchecked(index, std::move(obj));
				return true;
			}
			return false;
		}

		/// Adds an object to the grid without checking its coordinates. Use with caution.
		void add_object_at_unchecked(size_type index, T obj) {
			add_object_at_raw(_table.index_to_raw(index), std::move(obj));
		}
		/// \ref add_object_at() that takes a raw index.
		void add_object_at_raw(std::size_t index_raw, T obj) {
			_cell &cell = _table[index_raw];
			if (cell.count == 0) {
				_occupied_cells.emplace_back(index_raw);
			}
			std::size_t new_head = _chain.size();
			_chain.emplace_back(std::move(obj), cell.head);
			cell.head = new_head;
			++cell.count;
		}

		/// Returns the number of objects in the given cell.
		std::size_t get_num_objects_at(size_type index) const {
			return _table(index).count;
		}

		/// Clears all particles.
		void clear() {
			_table.fill(_cell());
			_chain.clear();
			_occupied_cells.clear();
		}

		/// Calls the given callback function for all objects near the given cell.
		template <typename Callback> void for_all_nearby_objects(
			vec3s cell, vec3s min_offset, vec3s max_offset, const Callback &cb
		) {
			vec3s min_corner, max_corner;
			vec_ops::apply_to(
				min_corner, [](std::size_t center, std::size_t offset) {
					return center < offset ? 0 : center - offset;
				}, cell, min_offset
			);
			max_offset += vec3s(1, 1, 1);
			vec_ops::apply_to(
				max_corner, [](std::size_t center, std::size_t offset, std::size_t max) {
					return std::min(center + offset, max);
				}, cell, max_offset, _table.get_size()
					);
			for (std::size_t z = min_corner.z; z < max_corner.z; ++z) {
				for (std::size_t y = min_corner.y; y < max_corner.y; ++y) {
					for (std::size_t x = min_corner.x; x < max_corner.x; ++x) {
						_cell &cell = _table(x, y, z);
						for (std::size_t i = 0, id = cell.head; i < cell.count; ++i) {
							_chain_elem &elem = _chain[id];
							cb(elem.object);
							id = elem.next;
						}
					}
				}
			}
		}

		/// Sorts and returns all fluid cells. This function will always sort the cells.
		std::vector<vec3s> get_sorted_occupied_cells() {
			std::sort(_occupied_cells.begin(), _occupied_cells.end());
			std::vector<vec3s> result;
			result.reserve(_occupied_cells.size());
			for (std::size_t sz : _occupied_cells) {
				result.emplace_back(_table.index_from_raw(sz));
			}
			return result;
		}
	private:
		/// The contents of a grid cell.
		struct _cell {
			std::size_t
				head = 0, ///< Index of the first element.
				count = 0; ///< The number of objects in this cell.
		};
		/// A node in the linked list.
		struct _chain_elem {
			/// Default constructor.
			_chain_elem() = default;
			/// Initializes all fields of this object.
			_chain_elem(T &&obj, std::size_t n) : object(std::move(obj)), next(n) {
			}

			T object; ///< The object.
			std::size_t next = 0; ///< Index of the next node.
		};
		using _grid_type = grid<Dim, _cell>; ///< The type of the grid.

		_grid_type _table; ///< The hash table.
		std::vector<_chain_elem> _chain; ///< The linked list.
		std::vector<std::size_t> _occupied_cells; ///< Stores all cells that contain objects.
	};
}
