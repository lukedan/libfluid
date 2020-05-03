#include "fluid/renderer/material.h"

/// \file
/// Implementation of materials.

#include "fluid/math/constants.h"
#include "fluid/math/warping.h"

namespace fluid::renderer {
	namespace materials {
		bsdf lambertian_reflection::get_bsdf(vec2d uv) const {
			bsdf result;
			auto &lambert = result.value.emplace<bsdfs::lambertian_reflection_brdf>();
			lambert.reflectance = reflectance.get_value_unit(uv);
			return result;
		}
	}


	bsdf material::get_bsdf(vec2d uv_unit) const {
		return std::visit(
			[&](const auto &mat) {
				bsdf result = mat.get_bsdf(uv_unit);
				result.emission = emission.get_value_unit(uv_unit);
				return result;
			},
			value
				);
	}
}
