#pragma once

/// \file
/// Matrices.

#include "vec.h"

namespace fluid {
	/// Row-major matrices.
	template <typename T, std::size_t W, std::size_t H, template <std::size_t, typename> typename VecT> struct rmat {
		constexpr static std::size_t
			width = W, ///< The number of columns in this matrix.
			height = H; ///< The number of rows in this matrix.
		using row_type = VecT<W, T>; ///< The type of a row.
		using column_type = VecT<H, T>; ///< The type of a column.
		using value_type = T; ///< The type of elements in this matrix.
		using storage_type = VecT<H, row_type>; ///< The type used to store the matrix.


		/// Zero-initializes this matrix.
		rmat() = default;
		/// Directly initializes this matrix from the storage.
		explicit rmat(const storage_type &storage) : rows(storage) {
		}

		/// Returns the identity matrix.
		[[nodiscard]] inline static rmat identity() {
			rmat result;
			std::size_t dim = 0;
			vec_ops::for_each(
				[&dim](row_type &row) {
					row[dim++] = static_cast<T>(1);
				},
				result.rows
					);
			return result;
		}


		// indexing
		/// Returns the element at the specified location.
		value_type &at(std::size_t x, std::size_t y) {
			assert(y < H);
			return rows[y][x];
		}
		/// \override
		value_type at(std::size_t x, std::size_t y) const {
			assert(y < H);
			return rows[y][x];
		}

		/// Returns the element at the specified location.
		value_type &operator()(std::size_t x, std::size_t y) {
			return at(x, y);
		}
		/// \overload
		value_type operator()(std::size_t x, std::size_t y) const {
			return at(x, y);
		}

		/// Returns the row at the given index.
		row_type row(std::size_t y) const {
			assert(y < H);
			return rows[y];
		}
		/// Returns the column at the given index.
		column_type column(std::size_t x) const {
			assert(x < W);
			return vec_ops::apply<column_type>(
				[x](const row_type &row) {
					return row[x];
				},
				rows
					);
		}


		// arithmetic
		/// In-place addition.
		rmat &operator+=(const rmat &rhs) {
			rows += rhs.rows;
			return *this;
		}
		/// Addition.
		[[nodiscard]] friend rmat operator+(const rmat &lhs, const rmat &rhs) {
			return rmat(lhs) += rhs;
		}

		/// In-place subtraction.
		rmat &operator-=(const rmat &rhs) {
			rows -= rhs.rows;
			return *this;
		}
		/// Subtraction.
		[[nodiscard]] friend rmat operator-(const rmat &lhs, const rmat &rhs) {
			return rmat(lhs) -= rhs;
		}
		/// Negation.
		[[nodiscard]] friend rmat operator-(const rmat &lhs) {
			return rmat(-lhs.rows);
		}

		/// In-place scalar division.
		rmat &operator/=(const T &rhs) {
			rows /= rhs;
			return *this;
		}
		/// Scalar division.
		[[nodiscard]] friend rmat operator/(rmat lhs, const T &rhs) {
			return lhs /= rhs;
		}

		/// In-place scalar multiplication.
		rmat &operator*=(const T &rhs) {
			rows *= rhs;
			return *this;
		}
		/// Scalar multiplication.
		[[nodiscard]] friend rmat operator*(rmat lhs, const T &rhs) {
			return lhs *= rhs;
		}
		/// Scalar multiplication.
		[[nodiscard]] friend rmat operator*(const T &lhs, rmat rhs) {
			return rhs *= lhs;
		}

		/// Matrix-vector multiplication.
		[[nodiscard]] friend column_type operator*(const rmat &lhs, const row_type &rhs) {
			return vec_ops::apply_to<column_type>(
				[](const row_type &lhs, const row_type &rhs) {
					return vec_ops::dot(lhs, rhs);
				},
				lhs, rhs
					);
		}


		// basic operations
		/// Returns the transposed matrix.
		rmat<T, H, W, VecT> transposed() const {
			using _trans_t = rmat<T, H, W, VecT>;

			_trans_t result;
			std::size_t dim = 0;
			vec_ops::for_each(
				[this, &dim](typename _trans_t::row_type &row) {
					row = column(dim++);
				},
				result.rows
			);
			return result;
		}


		storage_type rows; ///< The rows.
	};
}
