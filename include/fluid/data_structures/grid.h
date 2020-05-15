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
		grid(size_type size, const Cell &c) : _cells(get_array_size(size), c), _size(size) {
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
		template <typename Callback> void for_each(Callback &&cb) {
			_for_each_impl_wrapper(std::forward<Callback>(cb), size_type(), _size, std::make_index_sequence<Dim>());
		}
		/// Executes the given callback for the given range of the grid. The order in which the cells are visited is
		/// as close to the order in which they're stored in memory as possible. The range must completely lie inside
		/// the grid.
		template <typename Callback> void for_each_in_range_unchecked(Callback &&cb, size_type min, size_type max) {
			_for_each_impl_wrapper(std::forward<Callback>(cb), min, max, std::make_index_sequence<Dim>());
		}
		/// Executes the given callback for the given range of the grid.
		template <typename Callback> void for_each_in_range_checked(Callback &&cb, size_type min, size_type max) {
			vec_ops::for_each(
				[](std::size_t &coord, std::size_t max) {
					coord = std::min(coord, max);
				},
				max, _size
					);
			for_each_in_range_unchecked(std::forward<Callback>(cb), min, max);
		}
		/// Executes the given callback for the given range of the grid.
		template <typename Callback> void for_each_in_range_checked(
			Callback &&cb, size_type center, size_type diffmin, size_type diffmax
		) {
			size_type min_corner = vec_ops::apply<size_type>(
				[](std::size_t center, std::size_t offset) {
					return center < offset ? 0 : center - offset;
				}, center, diffmin
			);
			for_each_in_range_checked(std::forward<Callback>(cb), min_corner, center + diffmax + vec3s(1, 1, 1));
		}

		/// March through the grid. The callback's parameters are the (signed) index of the next cell, the direction
		/// (dimension) of the face that's hit, the surface normal, and the \p t value of the intersection point. The
		/// callback should return \p false to stop marching.
		template <typename Callback> void march_cells(Callback &&cb, vec<Dim, double> from, vec<Dim, double> to) {
			using _vecd = vec<Dim, double>;
			using _veci = vec<Dim, int>;

			auto _get_cell = [](_vecd pos) -> _veci {
				return vec_ops::apply<_veci>(
					[](double coord) {
						return static_cast<int>(std::floor(coord));
					},
					pos
						);
			};

			size_type coord;
			for (std::size_t i = 0; i < Dim; ++i) {
				coord[i] = i;
			}

			auto
				from_cell = _get_cell(from),
				to_cell = _get_cell(to);
			_vecd diff = to - from, inv_abs_diff;
			_veci advance, face_pos;
			vec_ops::for_each(
				[](int &adv, int &face, double &inv, double coord) {
					if (coord > 0.0) {
						adv = 1;
						face = 1;
					} else {
						adv = -1;
						face = 0;
					}
					inv = 1.0 / std::abs(coord);
				},
				advance, face_pos, inv_abs_diff, diff
					);
			_vecd normal = -_vecd(advance);

			auto t = vec_ops::memberwise::mul(
				vec_ops::apply<_vecd>(
					static_cast<double(*)(double)>(std::abs), _vecd(from_cell + face_pos) - from
					),
				inv_abs_diff
			);
			for (_veci current = from_cell; current != to_cell; ) {
				std::size_t min_coord = 0;
				double mint = 2.0;
				vec_ops::for_each(
					[&mint, &min_coord](double tv, std::size_t coord, std::size_t adv) {
						if (tv < mint) {
							mint = tv;
							min_coord = coord;
						}
					},
					t, coord, advance
						);
				if (!(mint <= 1.0)) {
					// emergency break - some floating point error has led us to outside of the path
					break;
				}

				current[min_coord] += advance[min_coord];
				_vecd n;
				n[min_coord] = normal[min_coord];
				if (!cb(current, min_coord, n, t[min_coord])) {
					break;
				}
				t[min_coord] += inv_abs_diff[min_coord];
			}
		}

		/// Converts a (x, y, z) position into a raw index for \ref _cells.
		std::size_t index_to_raw(size_type i) const {
			// hopefully the compiler will optimize away this empty check in the release build
			vec_ops::for_each(
				[](std::size_t coord, std::size_t size) {
					assert(coord < size);
				},
				i, _size
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
			Callback &&cb, size_type min, size_type max, std::index_sequence<Dims...>
		) {
			size_type cur = min;
			auto it = _cells.begin() + index_to_raw(min);
			_for_each_impl<(Dim - 1 - Dims)...>(std::forward<Callback>(cb), it, min, max, cur);
		}
		/// The implementation of the \ref for_each() function, for one dimension.
		template <
			std::size_t ThisDim, std::size_t ...OtherDims, typename Callback, typename It
		> FLUID_FORCEINLINE void _for_each_impl(
			Callback &&cb, const It &it, size_type min, size_type max, size_type &cur
		) const {
			if (min[ThisDim] >= max[ThisDim]) {
				return;
			}
			It my_it = it;
			std::size_t &i = cur[ThisDim];
			i = min[ThisDim];
			_for_each_impl<OtherDims...>(std::forward<Callback>(cb), my_it, min, max, cur);
			for (++i; i < max[ThisDim]; ++i) {
				my_it += _layer_offset[ThisDim];
				_for_each_impl<OtherDims...>(std::forward<Callback>(cb), my_it, min, max, cur);
			}
		}
		/// End of recursion, where the callback is actually invoked.
		template <typename Callback, typename It> FLUID_FORCEINLINE void _for_each_impl(
			Callback &&cb, It it, size_type, size_type, size_type cur
		) const {
			cb(cur, *it);
		}
	};

	template <typename Cell> using grid2 = grid<2, Cell>; ///< Shorthand for 2D grids.
	template <typename Cell> using grid3 = grid<3, Cell>; ///< Shorthand for 3D grids.
}
