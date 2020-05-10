#pragma once

/// \file
/// Functions used to warp samples from one domain to another.

#include "vec.h"

namespace fluid::warping {
	/// Maps a unit square uniformly to a unit disk.
	[[nodiscard]] vec2d unit_disk_from_unit_square(vec2d);
	/// Maps a unit square uniformly to a unit disk in a manner that is visually more uniform.
	[[nodiscard]] vec2d unit_disk_from_unit_square_concentric(vec2d);
	/// The probability density function of \ref unit_disk_from_unit_square_concentric().
	[[nodiscard]] double pdf_unit_disk_from_unit_square();

	/// Maps a unit square uniformly to a unit radius sphere.
	[[nodiscard]] vec3d unit_sphere_from_unit_square(vec2d);
	/// The probability density function of \ref unit_sphere_from_unit_square().
	[[nodiscard]] double pdf_unit_sphere_from_unit_square();

	/// Maps a unit square uniformly to a unit radius hemisphere.
	[[nodiscard]] vec3d unit_hemisphere_from_unit_square(vec2d);
	/// The probability density function of \ref unit_hemisphere_from_unit_square().
	[[nodiscard]] double pdf_unit_hemisphere_from_unit_square();

	/// Maps a unit square to a unit radius hemisphere. The mapping is weighed by the cosine of the angle between the
	/// output and the Y axis.
	[[nodiscard]] vec3d unit_hemisphere_from_unit_square_cosine(vec2d);
	/// The probability density function of \ref unit_hemisphere_from_unit_square_cosine().
	[[nodiscard]] double pdf_unit_hemisphere_from_unit_square_cosine(vec3d);
}
