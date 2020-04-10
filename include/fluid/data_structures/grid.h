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
			std::size_t mul = 1;
			vec_ops::for_each(
				[&mul](std::size_t &offset, std::size_t size) {
					offset = mul;
					mul *= size;
				},
				_layer_offset, get_size()
					);
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

		/// Executes the given callback for each cell in the grid. The order in which the cells are visited is the
		/// same as the order in which they're stored in memory.
		template <typename Callback> void for_each(Callback &cb) {
			_for_each_impl_wrapper(cb, size_type(), _size, std::make_index_sequence<Dim>());
		}
		/// Executes the given callback for the given range of the grid. The order in which the cells are visited is
		/// as close to the order in which they're stored in memory as possible.
		template <typename Callback> void for_each_in_range(Callback &cb, size_type min, size_type max) {
			_for_each_impl_wrapper(cb, min, max, std::make_index_sequence<Dim>());
		}

		/// Converts a (x, y, z) position into a raw index for \ref _cells.
		std::size_t index_to_raw(size_type i) const {
			// hopefully the compiler will optimize away this empty check in the release build
			vec_ops::for_each(
				[](std::size_t coord) {
					assert(coord < size);
				},
				i
					);
			return vec_ops::dot(i, _layer_offset);
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
		size_type
			_size, ///< The size of this grid.
			/// The offset that is used to obtain consecutive cells in a certain dimension. For example,
			/// \p _layer_offset[0] is 1, \p _layer_offset[1] is \p _size[0], \p _layer_offset[2] is
			/// <tt>_size[0] * _size[1]</tt>, and so on.
			_layer_offset;

		/// Wrapper around \ref _for_each_impl(). Call this with \p std::make_index_sequence<Dim>.
		template <std::size_t ...Dims, typename Callback> FLUID_FORCEINLINE void _for_each_impl_wrapper(
			Callback &cb, size_type min, size_type max, std::index_sequence<Dims...>
		) {
			size_type cur = min;
			auto it = _cells.begin() + index_to_raw(min);
			_for_each_impl<(Dim - 1 - Dims)...>(cb, it, min, max, cur);
		}
		/// The implementation of the \ref for_each() function, for one dimension.
		template <
			std::size_t ThisDim, std::size_t ...OtherDims, typename Callback, typename It
		> FLUID_FORCEINLINE void _for_each_impl(
			Callback &cb, const It &it, size_type min, size_type max, size_type &cur
		) const {
			It my_it = it;
			std::size_t &i = cur[ThisDim];
			for (i = min[ThisDim]; i < max[ThisDim]; ++i) {
				_for_each_impl<OtherDims...>(cb, my_it, min, max, cur);
				my_it += _layer_offset[ThisDim];
			}
		}
		/// End of recursion, where the callback is actually invoked.
		template <typename Callback, typename It> FLUID_FORCEINLINE void _for_each_impl(
			Callback &cb, It it, size_type min, size_type max, size_type cur
		) const {
			cb(cur, *it);
		}
	};

	template <typename Cell> using grid2 = grid<2, Cell>; ///< Shorthand for 2D grids.
	template <typename Cell> using grid3 = grid<3, Cell>; ///< Shorthand for 3D grids.
}
