#pragma once

/// \file
/// Implementation of the MAC grid.

#include <vector>

#include "../math/vec.h"

namespace fluid {
	/// A N-dimensional grid. The cells are stored in X-Y-Z- etc. order, i.e., cells with consecutive X coordinates
	/// are stored consecutively in memory.
	template <std::size_t Dim, typename Cell> class grid {
	public:
		using size_type = vec<Dim, std::size_t>; ///< The type used to store the size of this grid and indices.

		/// Default constructor.
		grid() = default;
		/// Initializes the cell storage.
		explicit grid(size_type size) : grid(size, Cell{}) {
		}
		/// Initializes the cell storage, seting all cells to the given value.
		grid(size_type size, const Cell &c) : _cells(get_array_size(size)), _size(size) {
		}

		/// Indexing.
		Cell &at(size_type i) {
			return _cells[index_to_raw(i)];
		}
		/// Indexing.
		const Cell &at(size_type i) const {
			return _cells[index_to_raw(i)];
		}
		/// Indexing.
		template <typename ...Args> std::enable_if_t<sizeof...(Args) == Dim, Cell> &operator()(Args &&...args) {
			return at(size_type(std::forward<Args>(args)...));
		}
		/// Indexing.
		Cell &operator()(size_type i) {
			return at(i);
		}
		/// Indexing.
		template <typename ...Args> std::enable_if_t<sizeof...(Args) == Dim, const Cell&> operator()(
			Args &&...args
		) const {
			return at(size_type(std::forward<Args>(args)...));
		}
		/// Indexing.
		const Cell &operator()(size_type i) const {
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
		size_type get_size() const {
			return _size;
		}

		/// Fills the entire grid using the given value.
		void fill(const Cell &value) {
			for (Cell &c : _cells) {
				c = value;
			}
		}

		/// Returns whether the cell at the given index is at the border of this grid.
		bool is_border_cell(size_type i) const {
			bool result = false;
			vec_ops::for_each(
				[&result](std::size_t val, std::size_t max) {
					if (val == 0 || val == max - 1) {
						result = true;
					}
				},
				i, get_size()
					);
			return result;
		}

		/// Converts a (x, y, z) position into a raw index for \ref _cells.
		std::size_t index_to_raw(size_type i) const {
			std::size_t total = 0, mul = 1;
			vec_ops::for_each(
				[&total, &mul](std::size_t coord, std::size_t size) {
					assert(coord < size);
					total += mul * coord;
					mul *= size;
				},
				i, get_size()
					);
			return total;
		}
		/// Converts a raw index for \ref _cells to a (x, y, z) position.
		size_type index_from_raw(std::size_t i) const {
			assert(i < get_array_size(get_size()));
			return vec_ops::apply<size_type>(
				[&i](std::size_t max) {
					std::size_t v = i % max;
					i /= max;
					return v;
				},
				get_size()
					);
		}

		/// Returns the array size given the grid size.
		inline static std::size_t get_array_size(size_type size) {
			std::size_t result = 1;
			vec_ops::for_each(
				[&result](std::size_t sc) {
					result *= sc;
				},
				size
					);
			return result;
		}
	private:
		std::vector<Cell> _cells; ///< Cell storage.
		size_type _size; ///< The size of this grid.
	};

	template <typename Cell> using grid2 = grid<2, Cell>; ///< Shorthand for 2D grids.
	template <typename Cell> using grid3 = grid<3, Cell>; ///< Shorthand for 3D grids.
}
