#pragma once

/// \file
/// Vector classes.

#include <cstddef>
#include <cmath>
#include <cassert>
#include <utility>
#include <optional>
#include <initializer_list>

namespace fluid {
	namespace vec_ops {
		/// Dot product.
		template <typename Vec> [[nodiscard]] inline typename Vec::value_type dot(
			const Vec &lhs, const Vec &rhs
		) {
			typename Vec::value_type result{};
			for (std::size_t i = 0; i < Vec::dimensionality; ++i) {
				result += lhs[i] * rhs[i];
			}
			return result;
		}
	}

	namespace _details {
		/// CRTP base for vectors. This class provides general functionalities of vectors.
		///
		/// \sa vec
		template <std::size_t N, typename T, typename Derived> struct vec_base {
			static_assert(N >= 2, "only vectors in two dimensions or above are allowed");

			constexpr static std::size_t dimensionality = N; ///< The dimensionality of this vector.
			using value_type = T; ///< The value type.

			/// Initializes this vector to zero.
			vec_base() = default;
			/// Initializes all components of this vector.
			template <typename ...Args, typename = std::enable_if_t<sizeof...(Args) == N>> vec_base(Args &&...args) {
				_init(std::make_index_sequence<sizeof...(Args)>(), std::forward<Args>(args)...);
			}
		private:
			/// Initializes all coordinates using the given arguments.
			template <std::size_t ...Is, typename ...Args> void _init(std::index_sequence<Is...>, Args &&...args) {
				_init_impl<Is...>(std::forward<Args>(args)...);
			}
			/// Implementation of \ref _init().
			template <std::size_t I, std::size_t ...OtherIs, typename Arg, typename ...OtherArgs> void _init_impl(
				Arg &&arg, OtherArgs &&...others
			) {
				_init_impl<I>(std::forward<Arg>(arg));
				_init_impl<OtherIs...>(std::forward<OtherArgs>(others)...);
			}
			/// Initializes a single element. Also serves as the end of recursion.
			template <std::size_t I, typename Arg> void _init_impl(Arg &&arg) {
				at(I) = std::forward<Arg>(arg);
			}
		public:
			/// Conversion from a vector of another type.
			template <typename U, typename D> explicit vec_base(const vec_base<N, U, D> &other) {
				for (std::size_t i = 0; i < N; ++i) {
					at(i) = static_cast<T>(other[i]);
				}
			}

		private:
			/// Casts this class to \p Derived.
			[[nodiscard]] Derived *_derived() {
				return static_cast<Derived*>(this);
			}
			/// Casts this class to \p Derived.
			[[nodiscard]] const Derived *_derived() const {
				return static_cast<const Derived*>(this);
			}

		public:
			// indexing
			/// Indexing.
			[[nodiscard]] T &at(std::size_t i) {
				assert(i < N);
				return _derived()->coord[i];
			}
			/// Indexing.
			[[nodiscard]] T at(std::size_t i) const {
				assert(i < N);
				return _derived()->coord[i];
			}
			/// Indexing.
			[[nodiscard]] T &operator[](std::size_t i) {
				return at(i);
			}
			/// Indexing.
			[[nodiscard]] T operator[](std::size_t i) const {
				return at(i);
			}


		private:
			/// Used as \p enable_if for non-template member functions.
			template <typename Ret, typename Dummy> using _valid_for_floating_point_t =
				std::enable_if_t<std::is_floating_point_v<T> == std::is_same_v<Dummy, void>, Ret>;

		public:
			// arithmetic
			/// In-place addition.
			Derived &operator+=(const Derived &rhs) {
				for (std::size_t i = 0; i < N; ++i) {
					at(i) += rhs[i];
				}
				return *_derived();
			}
			/// Addition.
			[[nodiscard]] friend Derived &operator+(const Derived &lhs, const Derived &rhs) {
				return Derived(lhs) += rhs;
			}

			/// In-place subtraction.
			Derived &operator-=(const Derived &rhs) {
				for (std::size_t i = 0; i < N; ++i) {
					at(i) -= rhs[i];
				}
				return *_derived();
			}
			/// Subtraction.
			[[nodiscard]] friend Derived &operator-(const Derived &lhs, const Derived &rhs) {
				return Derived(lhs) -= rhs;
			}

			/// In-place division.
			template <typename Dummy = void> _valid_for_floating_point_t<Derived&, Dummy> operator/=(T rhs) {
				for (std::size_t i = 0; i < N; ++i) {
					at(i) /= rhs;
				}
				return *_derived();
			}
			/// Division.
			template <typename Dummy = void> [[nodiscard]] friend _valid_for_floating_point_t<
				Derived&, Dummy
			> operator/(
				Derived lhs, T rhs
				) {
				return lhs /= rhs;
			}

			/// In-place multiplication.
			Derived &operator*=(T rhs) {
				for (std::size_t i = 0; i < N; ++i) {
					at(i) *= rhs;
				}
				return *_derived();
			}
			/// Multiplication.
			[[nodiscard]] friend Derived &operator*(Derived lhs, T rhs) {
				return lhs *= rhs;
			}
			/// Multiplication.
			[[nodiscard]] friend Derived &operator*(T lhs, Derived rhs) {
				return rhs *= lhs;
			}


			// length and normalization
			/// Returns the squared length of this vector.
			[[nodiscard]] T squared_length() const {
				return vec_ops::dot(*this, *this);
			}
			/// Returns the length of this vector.
			template <typename Dummy = void> [[nodiscard]] _valid_for_floating_point_t<T, Dummy> length() const {
				return std::sqrt(squared_length());
			}

			/// Returns the normalized vector and the length of the original vector.
			template <typename Dummy = void> [[nodiscard]] _valid_for_floating_point_t<
				std::pair<Derived, T>, Dummy
			> normalized_length_unchecked() const {
				T len = length();
				return { *this / len, len };
			}
			/// Returns the normalized vector.
			template <typename Dummy = void> [[nodiscard]] _valid_for_floating_point_t<
				Derived, Dummy
			> normalized_unchecked() const {
				return normalized_length_unchecked().first;
			}

			/// Returns the normalized vector and the length of the original vector. If the length is lower than the
			/// given threshold, this function returns nothing.
			template <typename Dummy = void> [[nodiscard]] _valid_for_floating_point_t<
				std::optional<std::pair<Derived, T>>, Dummy
			> normalized_length_checked(T eps = static_cast<T>(1e-6)) const {
				T sqlen = squared_length();
				if (sqlen <= eps * eps) {
					return std::nullopt;
				}
				sqlen = std::sqrt(sqlen);
				return std::pair<Derived, T>(*this / sqlen, sqlen);
			}
			/// Returns the normalized vector. If the length is lower than the given threshold, this function returns
			/// nothing.
			template <typename Dummy = void> [[nodiscard]] _valid_for_floating_point_t<
				std::optional<Derived>, Dummy
			> normalized_checked(T eps = static_cast<T>(1e-6)) const {
				if (auto result = normalized_length_checked(eps)) {
					return result->first;
				}
				return std::nullopt;
			}
		};
	}

	/// General vectors (5D or above).
	///
	/// \tparam N The dimensionality of this vector type.
	/// \tparam T The type of each coordinate.
	template <std::size_t N, typename T> struct vec : public _details::vec_base<N, T, vec<N, T>> {
		using vec_base::vec_base;

		T coord[N]{}; ///< All coordinates, zero-initialized.
	};

	/// 2D vectors.
	template <typename T> struct vec<2, T> : public _details::vec_base<2, T, vec<2, T>> {
		/// Initializes this vector to zero.
		vec() : x(), y() {
		}
		/// Initializes all coordinates.
		template <typename ...Args> vec(Args &&...args) : vec_base(std::forward<Args>(args)...) {
		}

		union {
			struct {
				T
					x, ///< The X component.
					y; ///< The Y component.
			};
			T coord[2]; ///< The coordinates.
		};
	};
	template <typename T> using vec2 = vec<2, T>; ///< Shorthand for 2D vectors.
	using vec2f = vec2<float>; ///< Shorthand for \p vec2<float>.
	using vec2d = vec2<double>; ///< Shorthand for \p vec2<double>.
	using vec2i = vec2<int>; ///< Shorthand for \p vec2<int>.

	/// 3D vectors.
	template <typename T> struct vec<3, T> : public _details::vec_base<3, T, vec<3, T>> {
		/// Initializes this vector to zero.
		vec() : x(), y(), z() {
		}
		/// Initializes all coordinates.
		template <typename ...Args> vec(Args &&...args) : vec_base(std::forward<Args>(args)...) {
		}

		union {
			struct {
				T
					x, ///< The X component.
					y, ///< The Y component.
					z; ///< The Z component.
			};
			T coord[3]; ///< The coordinates.
		};
	};
	template <typename T> using vec3 = vec<3, T>; ///< Shorthand for 3D vectors.
	using vec3f = vec3<float>; ///< Shorthand for \p vec3<float>.
	using vec3d = vec3<double>; ///< Shorthand for \p vec3<double>.
	using vec3i = vec3<int>; ///< Shorthand for \p vec3<int>.


	namespace vec_ops {
		/// Cross product.
		template <typename T> [[nodiscard]] inline vec3<T> cross(const vec3<T> &a, const vec3<T> &b) {
			return vec3<T>(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
		}
	}
}
