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
	template <typename T> FLUID_FORCEINLINE T lerp(const T &a, const T &b, double t) {
		return a * (1.0 - t) + b * t;
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
