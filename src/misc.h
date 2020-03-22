#pragma once

/// \file
/// Miscellaneous definitions.

#include <thread>
#include <mutex>

#ifdef _MSC_VER
#	define FLUID_FORCEINLINE __forceinline
#else
#	define FLUID_FORCEINLINE inline
#endif

namespace fluid {
	/// Linear interpolation.
	template <typename T> [[nodiscard]] FLUID_FORCEINLINE T lerp(const T &a, const T &b, double t) {
		return a * (1.0 - t) + b * t;
	}
	/// Bilinear interpolation.
	template <typename T> [[nodiscard]] FLUID_FORCEINLINE T bilerp(
		const T &v00, const T &v01, const T &v10, const T &v11, double t1, double t2
	) {
		return lerp(lerp(v00, v01, t2), lerp(v10, v11, t2), t1);
	}
	/// Trilinear interpolation.
	template <typename T> [[nodiscard]] FLUID_FORCEINLINE T trilerp(
		const T &v000, const T &v001, const T &v010, const T &v011,
		const T &v100, const T &v101, const T &v110, const T &v111,
		double t1, double t2, double t3
	) {
		return lerp(bilerp(v000, v001, v010, v011, t2, t3), bilerp(v100, v101, v110, v111, t2, t3), t1);
	}

	/// A semaphore. Copied from
	/// https://stackoverflow.com/questions/4792449/c0x-has-no-semaphores-how-to-synchronize-threads.
	class semaphore {
	public:
		void notify() {
			std::lock_guard<decltype(_mutex)> lock(_mutex);
			++_count;
			_cond.notify_one();
		}

		void wait() {
			std::unique_lock<decltype(_mutex)> lock(_mutex);
			while (_count == 0) { // handle spurious wake-ups
				_cond.wait(lock);
			}
			--_count;
		}
		bool try_wait() {
			std::lock_guard<decltype(_mutex)> lock(_mutex);
			if (_count > 0) {
				--_count;
				return true;
			}
			return false;
		}
	private:
		std::mutex _mutex;
		std::condition_variable _cond;
		unsigned _count = 0;

	};
}
