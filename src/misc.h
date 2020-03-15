#pragma once

/// \file
/// Miscellaneous definitions.

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
}
