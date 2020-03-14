#pragma once

/// \file
/// Implementation of the MAC grid.

#include <vector>

#include "../math/vec.h"

namespace fluid {
	/// A 3D grid. The cells are stored in X-Y-Z order, i.e., cells with consecutive X coordinates are stored
	/// consecutively in memory.
	template <typename Cell> class grid3 {
	public:
		/// Default constructor.
		grid3() = default;
		/// Initializes the cell storage.
		explicit grid3(vec3s size) : grid3(size, Cell{}) {
		}
		/// Initializes the cell storage, seting all cells to the given value.
		grid3(vec3s size, const Cell &c) : _cells(size.x * size.y * size.z), _size(size) {
		}

		/// Indexing.
		Cell &at(vec3s i) {
			return _cells[index_to_raw(i)];
		}
		/// Indexing.
		const Cell &at(vec3s i) const {
			return _cells[index_to_raw(i)];
		}
		/// Indexing.
		Cell &operator()(std::size_t x, std::size_t y, std::size_t z) {
			return at(vec3s(x, y, z));
		}
		/// Indexing.
		Cell &operator()(vec3s i) {
			return at(i);
		}
		/// Indexing.
		const Cell &operator()(std::size_t x, std::size_t y, std::size_t z) const {
			return at(vec3s(x, y, z));
		}
		/// Indexing.
		const Cell &operator()(vec3s i) const {
			return at(i);
		}

		/// Raw indexing.
		Cell &at_raw(std::size_t i) {
			return _cells[i];
		}
		/// Raw indexing.
		const Cell &at_raw(std::size_t i) const {
			return _cells[i];
		}
		/// Raw indexing.
		Cell &operator[](std::size_t i) {
			return at_raw(i);
		}
		/// Raw indexing.
		const Cell &operator[](std::size_t i) const {
			return at_raw(i);
		}

		/// Returns the size of this grid.
		vec3s get_size() const {
			return _size;
		}

		/// Fills the entire grid using the given value.
		void fill(const Cell &value) {
			for (Cell &c : _cells) {
				c = value;
			}
		}

		/// Returns whether the cell at the given index is at the border of this grid.
		bool is_border_cell(vec3s i) const {
			return
				i.x == 0 || i.x == _size.x - 1 ||
				i.y == 0 || i.y == _size.y - 1 ||
				i.z == 0 || i.z == _size.z - 1;
		}

		/// Converts a (x, y, z) position into a raw index for \ref _cells.
		std::size_t index_to_raw(vec3s i) const {
			assert(i.x < _size.x && i.y < _size.y && i.z < _size.z);
			return i.x + _size.x * (i.y + _size.y * i.z);
		}
		/// Converts a raw index for \ref _cells to a (x, y, z) position.
		vec3s index_from_raw(std::size_t i) const {
			assert(i < _cells.size());
			std::size_t yz = i / _size.x;
			return vec3s(i % _size.x, yz % _size.y, yz / _size.y);
		}
	private:
		std::vector<Cell> _cells; ///< Cell storage.
		vec3s _size; ///< The size of this grid.
	};
}
