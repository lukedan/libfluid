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


		bsdf specular_reflection::get_bsdf(vec2d uv) const {
			bsdf result;
			auto &specular = result.value.emplace<bsdfs::specular_reflection_brdf>();
			specular.reflectance = reflectance.get_value_unit(uv);
			return result;
		}


		bsdf specular_transmission::get_bsdf(vec2d uv) const {
			bsdf result;
			auto &specular = result.value.emplace<bsdfs::specular_transmission_bsdf>();
			specular.skin = skin.get_value_unit(uv);
			specular.index_of_refraction = index_of_refraction;
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
