#pragma once

/// \file
/// Short vectors.

#include <cstddef>
#include <cstdlib>
#include <cassert>
#include <utility>

namespace fluid {
	/// A short vector. The vector itself has capacity for \p Count elements, and if more elements are added all
	/// elements will be moved to heap-allocated memory.
	template <typename T, std::size_t Count> struct short_vec {
		template <typename, std::size_t> friend struct short_vec;
	public:
		using value_type = T; ///< Value type.
		using size_type = std::size_t; ///< Size type.
		using difference_type = std::ptrdiff_t; ///< Difference type.

		using reference = value_type&; ///< Reference type;
		using const_reference = const value_type&; ///< Const reference type;
		using pointer = value_type*; ///< Pointer type.
		using const_pointer = const value_type*; ///< Const pointer type.

		using iterator = T*; ///< Iterator type.
		using const_iterator = const T*; ///< Const iterator type.

		/// Initializes this short vector to be empty.
		short_vec() : _first(_local), _last(_local) {
		}
		/// Copy construction.
		template <std::size_t OtherCount> short_vec(const short_vec<T, OtherCount> &source) {
			size_type src_count = source.size();
			if (src_count <= Count) { // don't allocate
				_first = _local;
				_last = _local + src_count;
			} else { // allocate
				size_type cap = src_count;
				_first = _alloc(cap);
				_last = _capacity = _first + cap;
			}
			// copy elements
			auto src = source.begin();
			for (T *dst = _first; dst != _last; ++src, ++dst) {
				new (dst) T(*src);
			}
		}
		/// Move construction.
		template <std::size_t OtherCount> short_vec(short_vec<T, OtherCount> &&source) {
			if (!source._using_internal()) { // always take over memory
				_first = source._first;
				_last = source._last;
				_capacity = source._capacity;
			} else { // prefer using internal storage
				size_type src_count = source.size();
				if (src_count <= Count) { // use internal storage
					_first = _local;
					_last = _local + src_count;
				} else {
					size_type cap = src_count;
					_first = _alloc(cap);
					_last = _capacity = _first + cap;
				}
				// move elements
				for (T *src = source._first, *dst = _first; dst != _last; ++src, ++dst) {
					new (dst) T(std::move(*src));
					src->~T();
				}
			}
			source._first = source._last = source._local; // reset source
		}
		/// Copy assignment.
		template <std::size_t OtherCount> short_vec &operator=(const short_vec<T, OtherCount> &source) {
			// first destroy all existing elements
			for (T *cur = _first; cur != _last; ++cur) {
				cur->~T();
			}
			// decide where to store elements, result stored in _first
			size_type src_count = source.size();
			if (src_count <= Count) { // can use local storage
				if (!_using_internal()) { // has heap storage
					if (capacity() < src_count) { // not enough capacity, just use local instead
						_free(_first);
						_first = _local;
					} // otherwise use heap storage
				}
			} else {
				if (_using_internal()) { // allocate memory
					size_type cap = src_count;
					_first = _alloc(cap);
					_capacity = _first + cap;
				} else { // already has allocated heap memory
					if (capacity() < src_count) { // not enough space
						_free(_first);
						size_type cap = src_count;
						_first = _alloc(cap);
						_capacity = _first + cap;
					} // otherwise use old memory
				}
			}
			_last = _first + src_count; // adjust _last
			// copy elements
			auto src = source.begin();
			for (T *dst = _first; dst != _last; ++src, ++dst) {
				new (dst) T(*src);
			}
			return *this;
		}
		/// Move assignment.
		template <std::size_t OtherCount> short_vec &operator=(short_vec<T, OtherCount> &&source) {
			// first destroy all existing elements
			for (T *cur = _first; cur != _last; ++cur) {
				cur->~T();
			}
			if (source._using_internal()) {
				size_type src_count = source.size();
				if (src_count > capacity()) { // have to allocate new memory no matter what
					if (!_using_internal()) {
						_free(_first);
					}
					size_type cap = src_count;
					_first = _alloc(cap);
					_capacity = _first + cap;
				} // otherwise simply move elements over
				_last = _first + src_count; // update _last
				// move elements over
				for (T *src = source._first, *dst = _first; dst != _last; ++src, ++dst) {
					new (dst) T(std::move(*src));
					src->~T();
				}
			} else { // just take over
				if (!_using_internal()) {
					_free(_first);
				}
				_first = source._first;
				_last = source._last;
				_capacity = source._capacity;
			}
			source._first = source._last = source._local; // reset source
			return *this;
		}
		/// Destroys all elements and, if necessary, frees allocated memory.
		~short_vec() {
			for (T *cur = _first; cur != _last; ++cur) {
				cur->~T();
			}
			if (!_using_internal()) {
				_free(_first);
			}
		}

		/// Returns the size of this vector.
		size_type size() const {
			return _last - _first;
		}
		/// Returns the capacity of this vector.
		size_type capacity() const {
			return _using_internal() ? Count : _capacity - _first;
		}
		/// Returns whether this vector is empty.
		bool empty() const {
			return _first == _last;
		}


		// modification
		/// Constructs a new element in-place at the back of this vector.
		template <typename ...Args> reference emplace_back(Args &&...args) {
			size_type cur_size = size();
			if (cur_size == capacity()) { // full, needs extension
				size_type new_cap = _extended_capacity();
				T *new_array = _alloc(new_cap);
				// construct the new element first
				new (new_array + cur_size) T(std::forward<Args>(args)...);
				// then move the rest elements
				for (T *src = _first, *dst = new_array; src != _last; ++src, ++dst) {
					new (dst) T(std::move(*src));
					src->~T();
				}
				if (!_using_internal()) { // free previously allocated memory
					_free(_first);
				}
				// no member has been altered yet, start now
				_first = new_array;
				_last = new_array + cur_size;
				_capacity = new_array + new_cap;
			} else { // simply construct in-place
				new (_last) T(std::forward<Args>(args)...);
			}
			return *(_last++);
		}
		/// Returns the last element.
		reference back() {
			assert(!empty());
			return *(_last - 1);
		}
		/// \overload
		const_reference back() const {
			assert(!empty());
			return *(_last - 1);
		}
		/// Pops an element from the back of this vector.
		void pop_back() {
			assert(!empty());
			--_last;
			_last->~T();
			// TODO shrink?
		}
		/// Clears the contents of this vector without freeing heap memory.
		void clear() {
			for (T *cur = _first; cur != _last; ++cur) {
				cur->~T();
			}
			_last = _first;
		}


		// indexing
		/// Returns an iterator to the first element.
		iterator begin() {
			return _first;
		}
		/// Returns an iterator past the last element.
		iterator end() {
			return _last;
		}
		/// \overload
		const_iterator begin() const {
			return _first;
		}
		/// \overload
		const_iterator end() const {
			return _last;
		}

		/// Indexing.
		reference at(size_type i) {
			assert(i < size());
			return _first[i];
		}
		/// \overload
		const_reference at(size_type i) const {
			assert(i < size());
			return _first[i];
		}
		/// Indexing.
		reference operator[](size_type i) {
			return at(i);
		}
		/// \overload
		const_reference operator[](size_type i) const {
			return at(i);
		}
	private:
		union {
			T _local[Count]; ///< Elements that are stored on the stack.
			T *_capacity; ///< The capacity of (past) this vector, when using heap-allocated memory.
		};
		T
			*_first = nullptr, ///< Pointer to the first element.
			*_last = nullptr; ///< Pointer past the last element.

		/// Allocates memory for the specified number of elements.
		inline static T *_alloc(size_type count) {
#ifdef _MSC_VER
			return static_cast<T*>(::_aligned_malloc(sizeof(T) * count, alignof(T)));
#else
			return static_cast<T*>(std::aligned_alloc(alignof(T), sizeof(T) * count));
#endif
		}
		/// Frees memory.
		inline static void _free(T *ptr) {
#ifdef _MSC_VER
			::_aligned_free(ptr);
#else
			std::free(ptr);
#endif
		}

		/// Returns whether this vector is using the internal array.
		bool _using_internal() const {
			return _first == _local;
		}
		/// Returns the capacity if it were to be extended.
		size_type _extended_capacity() const {
			return static_cast<size_type>(capacity() * 1.5); // TODO magic number
		}
	};
}
