#pragma once

/// \file
/// Miscellaneous functions and definitions.

#include <maya/MStatus.h>

#define FLUID_MAYA_STR(X) #X
#define FLUID_MAYA_STR_EXPAND(X) FLUID_MAYA_STR(X)

/// Adds debug information to a message.
#define FLUID_DEBUG_MESSAGE(MSG) (__FILE__ ":" FLUID_MAYA_STR_EXPAND(__LINE__) " - " MSG)
/// Checks the given \p MStatus. The given object must be a variable name.
#define FLUID_MAYA_CHECK(MSTAT, MSG)                                   \
	if (!::fluid::maya::maya_check(MSTAT, FLUID_DEBUG_MESSAGE(MSG))) { \
		return MSTAT;                                                  \
	}
/// Checks the return value of the given function call which must be a \p MStatus.
#define FLUID_MAYA_CHECK_RETURN(FUNC_CALL, MSG) \
	{                                           \
		::MStatus _temp = (FUNC_CALL);          \
		FLUID_MAYA_CHECK(_temp, MSG);           \
	}

namespace fluid::maya {
	/// Checks the validity of the given \p MStatus.
	inline bool maya_check(const MStatus &stat, const char *text) {
		if (!stat) {
			stat.perror(text);
			return false;
		}
		return true;
	}
	/// \overload
	inline bool maya_check(const MStatus &stat) {
		return maya_check(stat, "undescribed maya error");
	}
}
