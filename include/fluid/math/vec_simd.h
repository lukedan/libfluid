#pragma once

/// \file
/// SIMD vector types.

#include <immintrin.h>

#include "vec.h"

namespace fluid {
	/// 4D double vectors implemented using AVX instructions. The lower bits contain the x value.
	struct vec4d_avx {
		/// Initializes \ref value.
		explicit vec4d_avx(__m256d v) : value(v) {
		}
		/// Sets all components of this vector. Intel did not give cycle counts for this operation.
		vec4d_avx(double x, double y, double z, double w) : value(_mm256_setr_pd(x, y, z, w)) {
		}

		/// Returns the zero vector.
		inline static vec4d_avx zero() {
			return vec4d_avx(_mm256_setzero_pd());
		}
		/// Loads data from four aligned doubles. The input values are stored in xyzw order.
		inline static vec4d_avx load_aligned(double arr[4]) {
			return vec4d_avx(_mm256_load_pd(arr));
		}
		/// Loads data from four doubles that may or may not be aligned. The input values are stored in xyzw order.
		inline static vec4d_avx load_unaligned(double arr[4]) {
			return vec4d_avx(_mm256_loadu_pd(arr));
		}
		/// Loads a single double value into all components of the vector.
		inline static vec4d_avx uniform(double v) {
			return vec4d_avx(_mm256_broadcast_sd(&v));
		}

		/// Returns the x coordinate.
		double x() const {
			return _mm256_cvtsd_f64(value);
		}
		/// Stores the contents of this vector into the given aligned array.
		void store_aligned(double arr[4]) const {
			_mm256_store_pd(arr, value);
		}
		/// Stores the contents of this vector into the given unaligned array.
		void store_unaligned(double arr[4]) const {
			_mm256_storeu_pd(arr, value);
		}
		/// Converts this vector to a \ref vec4d.
		explicit operator vec4d() const {
			alignas(__m256d) vec4d result;
			store_aligned(&result.x);
			return result;
		}


		/// Permutes the components of this vector and returns the result. Each template parameter indicates which
		/// component to put on that position, rather than where to put that component.
		template <int X, int Y, int Z, int W> vec4d_avx permuted() const {
			return vec4d_avx(_mm256_permute4x64_pd(value, _MM_SHUFFLE(W, Z, Y, X)));
		}


		// arithmetic
		/// Horizontal addition. The result is <tt>(lhs.x + lhs.y, rhs.x + rhs.y, lhs.z + lhs.w, rhs.z + rhs.w)</tt>.
		inline static vec4d_avx horizontal_add(vec4d_avx lhs, vec4d_avx rhs) {
			return vec4d_avx(_mm256_hadd_pd(lhs.value, rhs.value));
		}
		/// Horizontal subtraction. The result is
		/// <tt>(lhs.x - lhs.y, rhs.x - rhs.y, lhs.z - lhs.w, rhs.z - rhs.w)</tt>.
		inline static vec4d_avx horizontal_sub(vec4d_avx lhs, vec4d_avx rhs) {
			return vec4d_avx(_mm256_hsub_pd(lhs.value, rhs.value));
		}

		/// Addition.
		friend vec4d_avx operator+(vec4d_avx lhs, vec4d_avx rhs) {
			return vec4d_avx(_mm256_add_pd(lhs.value, rhs.value));
		}
		/// In-place addition.
		vec4d_avx &operator+=(vec4d_avx rhs) {
			return (*this) = (*this) + rhs;
		}
		/// Subtraction.
		friend vec4d_avx operator-(vec4d_avx lhs, vec4d_avx rhs) {
			return vec4d_avx(_mm256_sub_pd(lhs.value, rhs.value));
		}
		/// In-place subtraction.
		vec4d_avx &operator-=(vec4d_avx rhs) {
			return (*this) = (*this) - rhs;
		}

		__m256d value; ///< The value of this vector.
	};

	namespace vec_ops {
		namespace memberwise {
			/// Memberwise multiplication.
			inline vec4d_avx mul(vec4d_avx lhs, vec4d_avx rhs) {
				return vec4d_avx(_mm256_mul_pd(lhs.value, rhs.value));
			}
			/// Memberwise division.
			inline vec4d_avx div(vec4d_avx lhs, vec4d_avx rhs) {
				return vec4d_avx(_mm256_div_pd(lhs.value, rhs.value));
			}

			/// Square root.
			inline vec4d_avx sqrt(vec4d_avx value) {
				return vec4d_avx(_mm256_sqrt_pd(value.value));
			}

			/// Returns the minimum of each component.
			inline vec4d_avx min(vec4d_avx lhs, vec4d_avx rhs) {
				return vec4d_avx(_mm256_min_pd(lhs.value, rhs.value));
			}
			/// Returns the maximum of each component.
			inline vec4d_avx max(vec4d_avx lhs, vec4d_avx rhs) {
				return vec4d_avx(_mm256_max_pd(lhs.value, rhs.value));
			}

			/// Flooring.
			inline vec4d_avx floor(vec4d_avx v) {
				return vec4d_avx(_mm256_floor_pd(v.value));
			}
			/// Ceiling.
			inline vec4d_avx ceil(vec4d_avx v) {
				return vec4d_avx(_mm256_ceil_pd(v.value));
			}
			/// Rounding.
			inline vec4d_avx round(vec4d_avx v) {
				return vec4d_avx(_mm256_round_pd(v.value, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC));
			}
		}

		/// Dot product. This is less efficient than performing multiple dot products at once.
		inline double dot(vec4d_avx lhs, vec4d_avx rhs) {
			vec4d_avx res = memberwise::mul(lhs, rhs);
			res = vec4d_avx::horizontal_add(res, res);
			res = res.permuted<0, 2, 0, 2>();
			res = vec4d_avx::horizontal_add(res, res);
			return res.x();
		}
		/// Simultaneously computes the dot product of 4 pairs of vectors.
		inline vec4d_avx dot4(
			vec4d_avx a1, vec4d_avx b1, vec4d_avx a2, vec4d_avx b2,
			vec4d_avx a3, vec4d_avx b3, vec4d_avx a4, vec4d_avx b4
		) {
			vec4d_avx
				m1 = memberwise::mul(a1, b1), m2 = memberwise::mul(a2, b2),
				m3 = memberwise::mul(a3, b3), m4 = memberwise::mul(a4, b4);
			vec4d_avx s13 = vec4d_avx::horizontal_add(m1, m3), s24 = vec4d_avx::horizontal_add(m2, m4);
			return vec4d_avx::horizontal_add(s13.permuted<0, 2, 1, 3>(), s24.permuted<0, 2, 1, 3>());
		}

	}
}
