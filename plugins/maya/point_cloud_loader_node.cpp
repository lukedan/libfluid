#include "point_cloud_loader_node.h"

/// \file
/// Implementation of the point cloud loader node.

#include <fstream>

#include <maya/MFnTypedAttribute.h>
#include <maya/MFnPointArrayData.h>
#include <maya/MPointArray.h>

#include <fluid/data_structures/point_cloud.h>

#include "misc.h"

namespace fluid::maya {
	MObject
		point_cloud_loader_node::attr_file_name,
		point_cloud_loader_node::attr_output_points;
	MTypeId point_cloud_loader_node::id{ 0x98765433 };

	void *point_cloud_loader_node::creator() {
		return new point_cloud_loader_node();
	}

	MStatus point_cloud_loader_node::initialize() {
		MStatus stat;

		MFnTypedAttribute file_name;
		attr_file_name = file_name.create("fileName", "file", MFnData::kString, MObject::kNullObj, &stat);
		FLUID_MAYA_CHECK(stat, "parameter creation");

		MFnTypedAttribute output_points;
		attr_output_points = output_points.create(
			"outputPoints", "out", MFnData::kPointArray, MObject::kNullObj, &stat
		);
		FLUID_MAYA_CHECK(stat, "parameter creation");
		FLUID_MAYA_CHECK_RETURN(output_points.setWritable(false), "parameter creation");
		FLUID_MAYA_CHECK_RETURN(output_points.setStorable(false), "parameter creation");

		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_file_name), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_output_points), "parameter registration");

		FLUID_MAYA_CHECK_RETURN(attributeAffects(attr_file_name, attr_output_points), "parameter registration");

		return MStatus::kSuccess;
	}

	MStatus point_cloud_loader_node::compute(const MPlug &plug, MDataBlock &data_block) {
		if (plug == attr_output_points) {
			MStatus stat;

			MDataHandle file_name_data = data_block.inputValue(attr_file_name, &stat);
			FLUID_MAYA_CHECK(stat, "retrieve attribute");
			MDataHandle output_points_data = data_block.outputValue(attr_output_points, &stat);
			FLUID_MAYA_CHECK(stat, "retrieve attribute");

			MPointArray points;
			{
				std::ifstream fin(file_name_data.asString().asUTF8());
				point_cloud::load_from_naive(
					fin,
					[&points](vec3d p) {
						MStatus stat = points.append(MPoint(p.x, p.y, p.z));
						if (!stat) {
							stat.perror("MPointArray::append() failed");
						}
					}
				);
			}

			MFnPointArrayData points_array;
			MObject points_array_data = points_array.create(points, &stat);
			FLUID_MAYA_CHECK(stat, "finalize compute");
			FLUID_MAYA_CHECK_RETURN(output_points_data.set(points_array_data), "finalize compute");
			output_points_data.setClean();
			return MStatus::kSuccess;
		}
		return MStatus::kUnknownParameter;
	}
}
