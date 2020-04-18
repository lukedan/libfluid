#pragma once

/// \file
/// Vector classes.

#include <cstddef>
#include <cmath>
#include <cassert>
#include <utility>
#include <optional>
#include <functional>
#include <initializer_list>

#include "../misc.h"

namespace fluid {
	namespace vec_ops {
		namespace _details {
			/// Wrapper around \ref for_each_impl().
			template <std::size_t ...Is, typename Func, typename ...Vecs> inline void for_each(
				std::index_sequence<Is...>, const Func &func, Vecs &...vecs
			) {
				for_each_impl<Is...>(func, vecs...);
			}
			/// Implementation of \ref for_each().
			template <std::size_t I, std::size_t ...Is, typename Func, typename ...Vecs> inline void for_each_impl(
				const Func &func, Vecs &...vecs
			) {
				func(vecs[I]...);
				for_each_impl<Is...>(func, vecs...);
			}
			/// End of recursion.
			template <typename Func, typename ...Vecs> inline void for_each_impl(const Func&, Vecs&...) {
			}
		}
		/// Calls the given function on each member of each input vector in a Python <tt>zip</tt>-like fashion, with
		/// manual loop unrolling. This function requires that the \p FirstVec type has a static constexpr \p size()
		/// member. Use whenever possible.
		template <typename Func, typename FirstVec, typename ...OtherVecs> inline void for_each(
			const Func &func, FirstVec &first, OtherVecs &...others
		) {
			assert(((others.size() == FirstVec::size()) && ...));
			_details::for_each(std::make_index_sequence<FirstVec::size()>(), func, first, others...);
		}

		namespace _details {
			/// Wrapper around \ref apply_to_impl().
			template <
				std::size_t ...Is, typename Res, typename Func, typename ...Vecs
			> FLUID_FORCEINLINE void apply_to(
				std::index_sequence<Is...>, Res &out, const Func &func, const Vecs &...vecs
			) {
				apply_to_impl<Is...>(out, func, vecs...);
			}
			/// Implementation of \ref apply_to().
			template <
				std::size_t I, std::size_t ...Is, typename Res, typename Func, typename ...Vecs
			> FLUID_FORCEINLINE void apply_to_impl(Res &out, const Func &func, const Vecs &...vecs) {
				out[I] = func(vecs[I]...);
				apply_to_impl<Is...>(out, func, vecs...);
			}
			/// End of recursion.
			template <
				typename Res, typename Func, typename ...Vecs
			> FLUID_FORCEINLINE void apply_to_impl(Res&, const Func&, const Vecs&...) {
			}
		}
		/// Applies the given function to all elements of the input vectors, with manual loop unrolling. This
		/// function requires that the \p Res type has a static constexpr \p size() member. Use whenever possible.
		template <typename Res, typename Func, typename ...Vecs> inline void apply_to(
			Res &out, const Func &func, const Vecs &...vecs
		) {
			assert(((vecs.size() == Res::size()) && ...));
			_details::apply_to(std::make_index_sequence<Res::size()>(), out, func, vecs...);
		}

		/// Wrapper around \ref apply_to(). This version requires that the type has a static constexpr
		/// \p size() function.
		template <typename Res, typename Func, typename ...Vecs> [[nodiscard]] inline Res apply(
			const Func &func, const Vecs &...vecs
		) {
			assert(((vecs.size() == Res::size()) && ...));
			Res result;
			apply_to(result, func, vecs...);
			return result;
		}


		/// Dot product.
		template <typename Vec> [[nodiscard]] FLUID_FORCEINLINE typename Vec::value_type dot(
			const Vec &lhs, const Vec &rhs
		) {
			typename Vec::value_type result{};
			for_each(
				[&result](const typename Vec::value_type &a, const typename Vec::value_type &b) {
					result += a * b;
				},
				lhs, rhs
					);
			return result;
		}


		/// Memberwise vector operations.
		namespace memberwise {
			/// Memberwise multiplication.
			template <typename Vec> [[nodiscard]] FLUID_FORCEINLINE Vec mul(const Vec &lhs, const Vec &rhs) {
				return apply<Vec>(std::multiplies(), lhs, rhs);
			}

			/// Memberwise division.
			template <typename Vec> [[nodiscard]] FLUID_FORCEINLINE Vec div(const Vec &lhs, const Vec &rhs) {
				return apply<Vec>(std::divides(), lhs, rhs);
			}
		}


		/// Dynamic versions of vector operations.
		namespace dynamic {
			/// Calls the given function on each member of each input vector in a Python <tt>zip</tt>-like fashion.
			template <typename Func, typename FirstVec, typename ...OtherVecs> inline void for_each(
				const Func &func, FirstVec &first, OtherVecs &...others
			) {
				assert(((others.size() == first.size()) && ...));
				for (std::size_t i = 0; i < first.size(); ++i) {
					func(first[i], others[i]...);
				}
			}

			/// Applies the given function to all elements of the input vectors.
			template <typename Res, typename Func, typename ...Vecs> inline void apply_to(
				Res &out, const Func &func, const Vecs &...vecs
			) {
				assert(((vecs.size() == out.size()) && ...));
				for (std::size_t i = 0; i < out.size(); ++i) {
					out[i] = func(vecs[i]...);
				}
			}


			/// Dot product.
			template <typename Vec> [[nodiscard]] typename Vec::value_type dot(
				const Vec &lhs, const Vec &rhs
			) {
				assert(lhs.size() == rhs.size());
				typename Vec::value_type result{};
				for (std::size_t i = 0; i < lhs.size(); ++i) {
					result += lhs[i] * rhs[i];
				}
				return result;
			}
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
		protected:
			/// Initializes all coordinates using the given arguments. Concrete \p vec structs should use this to
			/// implement direct initialization.
			template <std::size_t ...Is, typename ...Args> void _init(std::index_sequence<Is...>, Args &&...args) {
				_init_impl<Is...>(std::forward<Args>(args)...);
			}
		private:
			/// Implementation of \ref _init().
			template <std::size_t I, std::size_t ...OtherIs, typename Arg, typename ...OtherArgs> void _init_impl(
				Arg &&arg, OtherArgs &&...others
			) {
				_init_impl<I>(std::forward<Arg>(arg));
				_init_impl<OtherIs...>(std::forward<OtherArgs>(others)...);
			}
			/// Initializes a single element. Also serves as the end of recursion.
			template <std::size_t I, typename Arg> void _init_impl(Arg &&arg) {
				(*_derived())[I] = std::forward<Arg>(arg);
			}
		protected:
			/// Conversion from a vector of another type. Concrete \p vec structs should use this to implement
			/// conversion.
			template <typename Other> void _convert(const Other &other) {
				vec_ops::apply_to(
					*_derived(),
					[](const typename Other::value_type &v) {
						return static_cast<T>(v);
					},
					other
						);
			}

		public:
			/// Returns the unit vector that corresponds to the \p I-th axis.
			template <std::size_t I> inline constexpr static Derived axis() {
				static_assert(I < N, "invalid axis");
				Derived result;
				result[I] = static_cast<T>(1);
				return result;
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
			[[nodiscard]] T &operator[](std::size_t i) {
				return _derived()->at(i);
			}
			/// Indexing.
			[[nodiscard]] T operator[](std::size_t i) const {
				return _derived()->at(i);
			}


			// pretend that this is a std::vector
			constexpr inline static std::size_t size() {
				return dimensionality;
			}
			// TODO begin(), end(), etc. if necessary


		private:
			/// Used as \p enable_if for non-template member functions.
			template <typename Ret, typename Dummy> using _valid_for_floating_point_t =
				std::enable_if_t<std::is_floating_point_v<T> == std::is_same_v<Dummy, void>, Ret>;
			/// Used as \p enable_if for non-template member functions.
			template <typename Ret, typename Dummy> using _valid_for_integral_t =
				std::enable_if_t<std::is_integral_v<T> == std::is_same_v<Dummy, void>, Ret>;
		public:
			// arithmetic
			/// In-place addition.
			FLUID_FORCEINLINE Derived &operator+=(const Derived &rhs) {
				vec_ops::apply_to(*_derived(), std::plus(), *_derived(), rhs);
				return *_derived();
			}
			/// Addition.
			FLUID_FORCEINLINE [[nodiscard]] friend Derived operator+(const Derived &lhs, const Derived &rhs) {
				return Derived(lhs) += rhs;
			}

			/// In-place subtraction.
			FLUID_FORCEINLINE Derived &operator-=(const Derived &rhs) {
				vec_ops::apply_to(*_derived(), std::minus(), *_derived(), rhs);
				return *_derived();
			}
			/// Subtraction.
			FLUID_FORCEINLINE [[nodiscard]] friend Derived operator-(const Derived &lhs, const Derived &rhs) {
				return Derived(lhs) -= rhs;
			}
			/// Negation.
			FLUID_FORCEINLINE [[nodiscard]] friend Derived operator-(const Derived &lhs) {
				return vec_ops::apply<Derived>(std::negate(), lhs);
			}

			/// In-place division.
			template <typename Dummy = void> FLUID_FORCEINLINE _valid_for_floating_point_t<
				Derived&, Dummy
			> operator/=(const T &rhs) {
				vec_ops::apply_to(
					*_derived(), [&](const T &lhs) {
						return lhs / rhs;
					}, *_derived()
						);
				return *_derived();
			}
			/// Division.
			template <typename Dummy = void> FLUID_FORCEINLINE [[nodiscard]] friend _valid_for_floating_point_t<
				Derived, Dummy
			> operator/(Derived lhs, const T &rhs) {
				return lhs /= rhs;
			}

			/// In-place multiplication.
			FLUID_FORCEINLINE Derived &operator*=(const T &rhs) {
				vec_ops::apply_to(
					*_derived(), [&](const T &lhs) {
						return lhs * rhs;
					}, *_derived()
						);
				return *_derived();
			}
			/// Multiplication.
			FLUID_FORCEINLINE [[nodiscard]] friend Derived operator*(Derived lhs, const T &rhs) {
				return lhs *= rhs;
			}
			/// Multiplication.
			FLUID_FORCEINLINE [[nodiscard]] friend Derived operator*(const T &lhs, Derived rhs) {
				return rhs *= lhs;
			}


			// comparison
			/// Equality.
			template <typename Dummy = void> FLUID_FORCEINLINE [[nodiscard]] friend _valid_for_integral_t<
				bool, Dummy
			> operator==(const Derived &lhs, const Derived &rhs) {
				bool res = true;
				vec_ops::for_each(
					[&res](T lhs, T rhs) {
						if (lhs != rhs) {
							res = false;
						}
					},
					lhs, rhs
						);
				return res;
			}
			/// Inequality.
			template <typename Dummy = void> FLUID_FORCEINLINE [[nodiscard]] friend _valid_for_integral_t<
				bool, Dummy
			> operator!=(const Derived &lhs, const Derived &rhs) {
				return !(lhs == rhs);
			}


			// length and normalization
			/// Returns the squared length of this vector.
			[[nodiscard]] T squared_length() const {
				return vec_ops::dot(*_derived(), *_derived());
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
				return { *_derived() / len, len };
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
				return std::pair<Derived, T>(*_derived() / sqlen, sqlen);
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
		/// Default constructor.
		vec() = default;
		/// Direct initialization.
		template <typename ...Args, typename = std::enable_if_t<sizeof...(Args) == N>> vec(Args &&...args) {
			this->_init(std::make_index_sequence<N>(), std::forward<Args>(args)...);
		}
		/// Conversion.
		template <typename Other> explicit vec(const vec<N, Other> &src) {
			this->_convert(src);
		}

		/// Indexing.
		FLUID_FORCEINLINE [[nodiscard]] T &at(std::size_t i) {
			assert(i < N);
			return v[i];
		}
		/// Indexing.
		FLUID_FORCEINLINE [[nodiscard]] T at(std::size_t i) const {
			assert(i < N);
			return v[i];
		}

		T v[N]{}; ///< All coordinates, zero-initialized.
	};


	/// 2D vectors.
	template <typename T> struct vec<2, T> : public _details::vec_base<2, T, vec<2, T>> {
		/// Initializes this vector to zero.
		vec() = default;
		/// Direct initialization.
		template <typename ...Args, typename = std::enable_if_t<sizeof...(Args) == 2>> vec(Args &&...args) {
			this->_init(std::make_index_sequence<2>(), std::forward<Args>(args)...);
		}
		/// Conversion.
		template <typename Other> explicit vec(const vec<2, Other> &src) {
			this->_convert(src);
		}

		/// Indexing.
		FLUID_FORCEINLINE [[nodiscard]] T &at(std::size_t i) {
			assert(i < 2);
			return (&x)[i];
		}
		/// Indexing.
		FLUID_FORCEINLINE [[nodiscard]] T at(std::size_t i) const {
			assert(i < 2);
			return (&x)[i];
		}

		T
			x{}, ///< The X component.
			y{}; ///< The Y component.
	};
	template <typename T> using vec2 = vec<2, T>; ///< Shorthand for 2D vectors.
	using vec2f = vec2<float>; ///< Shorthand for \p vec2<float>.
	using vec2d = vec2<double>; ///< Shorthand for \p vec2<double>.
	using vec2i = vec2<int>; ///< Shorthand for \p vec2<int>.
	using vec2s = vec2<std::size_t>; ///< Shorthand for \p vec2<std::size_t>.


	/// 3D vectors.
	template <typename T> struct vec<3, T> : public _details::vec_base<3, T, vec<3, T>> {
		/// Initializes this vector to zero.
		vec() = default;
		/// Direct initialization.
		template <typename ...Args, typename = std::enable_if_t<sizeof...(Args) == 3>> vec(Args &&...args) {
			this->_init(std::make_index_sequence<3>(), std::forward<Args>(args)...);
		}
		/// Conversion.
		template <typename Other> explicit vec(const vec<3, Other> &src) {
			this->_convert(src);
		}

		/// Indexing.
		FLUID_FORCEINLINE [[nodiscard]] T &at(std::size_t i) {
			assert(i < 3);
			return (&x)[i];
		}
		/// Indexing.
		FLUID_FORCEINLINE [[nodiscard]] T at(std::size_t i) const {
			assert(i < 3);
			return (&x)[i];
		}

		T
			x{}, ///< The X component.
			y{}, ///< The Y component.
			z{}; ///< The Z component.
	};
	template <typename T> using vec3 = vec<3, T>; ///< Shorthand for 3D vectors.
	using vec3f = vec3<float>; ///< Shorthand for \p vec3<float>.
	using vec3d = vec3<double>; ///< Shorthand for \p vec3<double>.
	using vec3i = vec3<int>; ///< Shorthand for \p vec3<int>.
	using vec3s = vec3<std::size_t>; ///< Shorthand for \p vec3<std::size_t>.


	/// 4D vectors.
	template <typename T> struct vec<4, T> : public _details::vec_base<4, T, vec<4, T>> {
		/// Initializes this vector to zero.
		vec() = default;
		/// Direct initialization.
		template <typename ...Args, typename = std::enable_if_t<sizeof...(Args) == 4>> vec(Args &&...args) {
			this->_init(std::make_index_sequence<4>(), std::forward<Args>(args)...);
		}
		/// Conversion.
		template <typename Other> explicit vec(const vec<4, Other> &src) {
			this->_convert(src);
		}

		/// Indexing.
		FLUID_FORCEINLINE [[nodiscard]] T &at(std::size_t i) {
			assert(i < 4);
			return (&x)[i];
		}
		/// Indexing.
		FLUID_FORCEINLINE [[nodiscard]] T at(std::size_t i) const {
			assert(i < 4);
			return (&x)[i];
		}

		T
			x{}, ///< The X component.
			y{}, ///< The Y component.
			z{}, ///< The Z component.
			w{}; ///< The W component.
	};
	template <typename T> using vec4 = vec<4, T>; ///< Shorthand for 4D vectors.
	using vec4f = vec4<float>; ///< Shorthand for \p vec4<float>.
	using vec4d = vec4<double>; ///< Shorthand for \p vec4<double>.
	using vec4i = vec4<int>; ///< Shorthand for \p vec4<int>.


	namespace vec_ops {
		/// Cross product.
		template <typename T> [[nodiscard]] inline vec3<T> cross(const vec3<T> &a, const vec3<T> &b) {
			return vec3<T>(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
		}
	}
}
