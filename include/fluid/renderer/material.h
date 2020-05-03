#pragma once

/// \file
/// Definition of materials.

#include <variant>

#include "common.h"
#include "bsdf.h"

namespace fluid::renderer {
	/// Definition of different materials.
	namespace materials {
		/// A *channel* stores a shared pointer to a texture and a value used to modulate the texture. If the texture
		/// is \p null, then it's assumed to be pure white.
		///
		/// \todo Wrapping modes.
		template <typename T> struct channel {
			std::shared_ptr<image<T>> texture; ///< The texture.
			T modulation; ///< The spectrum that the texture is multiplied by.

			/// Returns the value of this channel at the given texture coordinates. The coordinates are assumed to be
			/// within [0, 1].
			spectrum get_value_unit(vec2d uv) const {
				if (texture == nullptr) {
					return modulation;
				}
				return modulate(texture->sample_unit(uv), modulation);
			}
		};

		/// Lambertian reflection material.
		struct lambertian_reflection {
			channel<spectrum> reflectance; ///< Reflectance.

			/// Returns a \ref bsdfs::lambertian_reflection_brdf.
			bsdf get_bsdf(vec2d) const;
		};
	}

	/// A generic material.
	struct material {
		/// The \p std::variant used to store the material.
		using union_t = std::variant<materials::lambertian_reflection>;

		/// Returns a BSDF that corresponds to the given UV coordinates. The UV coordinates are assumed to be within
		/// [0, 1]. \ref bsdf::emission is set by this function, while \ref bsdf::value is set by the underlying
		/// material type.
		bsdf get_bsdf(vec2d) const;

		union_t value; ///< The value of this material.
		materials::channel<spectrum> emission; ///< The emission of this material.
	};
}
