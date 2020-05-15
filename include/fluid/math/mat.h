#pragma once

/// \file
/// Matrices.

#include "vec.h"

namespace fluid {
	/// Row-major matrices.
	template <
		typename T, std::size_t W, std::size_t H, template <std::size_t, typename> typename VecT = vec
	> struct rmat {
		constexpr static std::size_t
			width = W, ///< The number of columns in this matrix.
			height = H; ///< The number of rows in this matrix.
		constexpr static bool is_square = W == H; ///< Indicates whether this is a square matrix.

		using row_type = VecT<W, T>; ///< The type of a row.
		using column_type = VecT<H, T>; ///< The type of a column.
		using value_type = T; ///< The type of elements in this matrix.
		using storage_type = VecT<H, row_type>; ///< The type used to store the matrix.


		/// Zero-initializes this matrix.
		rmat() = default;
	private:
		/// Directly initializes this matrix from the storage.
		explicit rmat(const storage_type &mat_rows) : rows(mat_rows) {
		}

	public:
		/// Creates a new \ref rmat from its rows.
		template <typename ...Args> [[nodiscard]] inline static rmat from_rows(Args &&...args) {
			return rmat(storage_type(std::forward<Args>(args)...));
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
			return vec_ops::apply<column_type>(
				[&rhs](const row_type &row) {
					return vec_ops::dot(row, rhs);
				},
				lhs.rows
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

		/// Returns the submatrix that is used to calculate the minor of this matrix, i.e., this matrix without the
		/// \p er-th row and \p ec-th column.
		rmat<T, H - 1, W - 1, VecT> get_submatrix_minor(std::size_t er, std::size_t ec) const {
			using _result_t = rmat<T, H - 1, W - 1, VecT>;
			_result_t result;
			std::size_t rid = 0;
			vec_ops::for_each(
				[&](typename _result_t::row_type &row) {
					row_type myrow = rows[rid >= er ? rid + 1 : rid];
					std::size_t cid = 0;
					vec_ops::for_each(
						[&](T &col) {
							col = myrow[cid >= ec ? cid + 1 : cid];
							++cid;
						},
						row
							);
					++rid;
				},
				result.rows
					);
			return result;
		}

	private:
		/// Shorthand for \p std::enable_if_t.
		template <bool Enable, typename U, typename Dummy> using _enable_if_t =
			std::enable_if_t<Enable == std::is_same_v<Dummy, void>, U>;
		/// \p std::enable_if_t for square matrices.
		template <typename U, typename Dummy> using _enable_if_square_t = _enable_if_t<is_square, U, Dummy>;
	public:
		/// Calculates the determinant of this matrix.
		template <typename Dummy = void> _enable_if_square_t<T, Dummy> get_determinant() const {
			if constexpr (W == 1) {
				return rows[0][0];
			} else {
				if constexpr (W == 2) { // termination condition
					return rows[0][0] * rows[1][1] - rows[0][1] * rows[1][0];
				} else {
					T result = static_cast<T>(0);
					bool positive = true;
					std::size_t rid = 0;
					vec_ops::for_each(
						[&](const row_type &row) {
							T value = row[0] * get_submatrix_minor(rid, 0).get_determinant();
							result = positive ? result + value : result - value;
							++rid;
							positive = !positive;
						},
						rows
							);
					return result;
				}
			}
		}

		/// Calculates the inverse matrix.
		template <typename Dummy = void> _enable_if_square_t<rmat, Dummy> get_inverse() const {
			rmat result;
			// compute adjugate
			std::size_t rid = 0;
			bool rinv = false;
			vec_ops::for_each(
				[&](row_type &row) {
					std::size_t cid = 0;
					bool cinv = rinv;
					vec_ops::for_each(
						[&](T &col) {
							// transposed here
							col = get_submatrix_minor(cid, rid).get_determinant();
							if (cinv) {
								col = -col;
							}
							++cid;
							cinv = !cinv;
						},
						row
							);
					++rid;
					rinv = !rinv;
				},
				result.rows
					);
			return result / get_determinant();
		}


		storage_type rows; ///< The rows.
	};

	template <
		typename T, std::size_t H1, std::size_t W1H2, std::size_t W2,
		template <std::size_t, typename> typename VecT
	> inline rmat<T, W2, H1, VecT> operator*(
		const rmat<T, W1H2, H1, VecT> &lhs, const rmat<T, W2, W1H2, VecT> &rhs
	) {
		using _lhs_t = rmat<T, W1H2, H1, VecT>;
		using _rhs_trans_t = rmat<T, W1H2, W2, VecT>;
		using _result_t = rmat<T, W2, H1, VecT>;

		_rhs_trans_t rhs_trans = rhs.transposed();
		_result_t result;
		vec_ops::for_each(
			[&](typename _result_t::row_type &res_row, const typename _lhs_t::row_type &lhs_row) {
				vec_ops::for_each(
					[&](T &dest, const typename _rhs_trans_t::row_type &rhs_row) {
						dest = vec_ops::dot(lhs_row, rhs_row);
					},
					res_row, rhs_trans.rows
						);
			},
			result.rows, lhs.rows
				);
		return result;
	}

	template <typename T> using rmat3 = rmat<T, 3, 3>; ///< Shorthand for 3x3 matrix types.
	using rmat3d = rmat3<double>; ///< Double 3x3 matrices.

	template <typename T> using rmat4 = rmat<T, 4, 4>; ///< Shorthand for 4x4 matrix types.
	using rmat4d = rmat4<double>; ///< Double 4x4 matrices.

	template <typename T> using rmat3x4 = rmat<T, 4, 3>; ///< Shorthand for 3x4 matrix types.
	using rmat3x4d = rmat3x4<double>; ///< Double 3x4 matrices.


	namespace transform {
		/// Returns a scaling matrix.
		inline rmat3d scale(vec3d s) {
			rmat3d result;
			result(0, 0) = s.x;
			result(1, 1) = s.y;
			result(2, 2) = s.z;
			return result;
		}
		/// Rotation using Euler angles using Z -> Y -> X as the rotation order.
		inline rmat3d rotate_euler(vec3d angle) {
			double
				sx = std::sin(angle.x), cx = std::cos(angle.x),
				sy = std::sin(angle.y), cy = std::cos(angle.y),
				sz = std::sin(angle.z), cz = std::cos(angle.z);

			rmat3d result;

			result(0, 0) = cy * cz;
			result(1, 0) = sx * sy * cz - cx * sz;
			result(2, 0) = cx * sy * cz + sx * sz;

			result(0, 1) = cy * sz;
			result(1, 1) = sx * sy * sz + cx * cz;
			result(2, 1) = cx * sy * sz - sx * cz;

			result(0, 2) = -sy;
			result(1, 2) = cy * sx;
			result(2, 2) = cy * cx;

			return result;
		}

		/// Scales, rotates, and then translates the coordinate system.
		inline rmat3x4d scale_rotate_translate(vec3d sc, vec3d euler, vec3d translate) {
			rmat3d left = rotate_euler(euler) * scale(sc);
			rmat3x4d result;

			result(0, 0) = left(0, 0);
			result(1, 0) = left(1, 0);
			result(2, 0) = left(2, 0);
			result(3, 0) = translate.x;

			result(0, 1) = left(0, 1);
			result(1, 1) = left(1, 1);
			result(2, 1) = left(2, 1);
			result(3, 1) = translate.y;

			result(0, 2) = left(0, 2);
			result(1, 2) = left(1, 2);
			result(2, 2) = left(2, 2);
			result(3, 2) = translate.z;

			return result;
		}
	}
}
