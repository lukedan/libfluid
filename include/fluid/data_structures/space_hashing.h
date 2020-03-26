#pragma once

/// \file
/// Definition of the space hashing data structure.

#include <vector>
#include <algorithm>

#include "grid.h"
#include "particle.h"

namespace fluid {
	/// A space-hashing data structure. Each grid stores particles in that grid.
	template <typename T> struct space_hashing {
	public:
		/// Default constructor.
		space_hashing() = default;
		/// Initializes the table to the given size.
		explicit space_hashing(vec3s size) : _table(size) {
		}

		/// Resizes the grid, clearing all data in the process.
		void resize(vec3s size) {
			_table = grid3<std::vector<T*>>(size);
			_occupied_cells.clear();
		}

		/// Adds an object to the grid. The object must not be moved in memory after it has been added until
		/// \ref clear() has been called. The object will not be added if it falls out of the grid.
		///
		/// \return Whether the object is in the grid and has been added.
		bool add_object_at(vec3s index, T &obj) {
			if (index.x < _table.get_size().x && index.y < _table.get_size().y && index.z < _table.get_size().z) {
				add_object_at_unchecked(index, obj);
				return true;
			}
			return false;
		}
		/// Adds an object to the grid without checking its coordinates. Use with caution.
		void add_object_at_unchecked(vec3s index, T &obj) {
			add_object_at_raw(_table.index_to_raw(index), obj);
		}
		/// \ref add_object_at() that takes a raw index.
		void add_object_at_raw(std::size_t index_raw, T &obj) {
			std::vector<T*> &cell = _table[index_raw];
			if (cell.empty()) {
				_occupied_cells.emplace_back(index_raw);
			}
			cell.emplace_back(&obj);
		}
		/// Returns all objects in the given cell.
		const std::vector<T*> &get_objects_at(vec3s index) const {
			return _table(index);
		}
		/// Clears all particles.
		void clear() {
			resize(_table.get_size());
		}

		/// Calls the given callback function for all objects near the given cell.
		template <typename Callback> void for_all_nearby_objects(
			vec3s cell, vec3s min_offset, vec3s max_offset, const Callback &cb
		) const {
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
						for (T *obj : _table(x, y, z)) {
							cb(*obj);
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
		grid3<std::vector<T*>> _table; ///< The hash table.
		std::vector<std::size_t> _occupied_cells; ///< Stores all cells that contain objects.
	};
}
