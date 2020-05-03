#include "voxelizer_node.h"

/// \file
/// Implementation of the source node.

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMesh.h>
#include <maya/MPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MFnIntArrayData.h>

#include <fluid/voxelizer.h>

#include "../misc.h"

namespace fluid::maya {
	MObject
		voxelizer_node::attr_input_mesh,
		voxelizer_node::attr_cell_size,
		voxelizer_node::attr_ref_grid_offset,
		voxelizer_node::attr_ref_grid_size,
		voxelizer_node::attr_include_interior,
		voxelizer_node::attr_include_surface,

		voxelizer_node::attr_output_grid_offset,
		voxelizer_node::attr_output_grid_size,
		voxelizer_node::attr_output_cells,
		voxelizer_node::attr_output_cells_ref;

	void *voxelizer_node::creator() {
		return new voxelizer_node();
	}

	MStatus voxelizer_node::initialize() {
		MStatus stat;

		MFnTypedAttribute input_mesh;
		attr_input_mesh = input_mesh.create(
			"inputTriangleMesh", "mesh", MFnData::kMesh, MObject::kNullObj, &stat
		);
		FLUID_MAYA_CHECK(stat, "parameter creation");

		MFnNumericAttribute cell_size;
		attr_cell_size = cell_size.create("cellSize", "cell", MFnNumericData::kDouble, 0.5, &stat);
		FLUID_MAYA_CHECK(stat, "parameter creation");

		MFnNumericAttribute ref_grid_offset;
		attr_ref_grid_offset = ref_grid_offset.create(
			"referenceGridOffset", "roff", MFnNumericData::k3Double, 0.0, &stat
		);
		FLUID_MAYA_CHECK(stat, "parameter creation");

		MFnNumericAttribute ref_grid_size;
		attr_ref_grid_size = ref_grid_size.create("referenceGridSize", "rsz", MFnNumericData::k3Int, 50.0, &stat);
		FLUID_MAYA_CHECK(stat, "parameter creation");

		MFnNumericAttribute include_interior;
		attr_include_interior = include_interior.create(
			"includeInterior", "int", MFnNumericData::kBoolean, 1.0, &stat
		);
		FLUID_MAYA_CHECK(stat, "parameter creation");

		MFnNumericAttribute include_surface;
		attr_include_surface = include_surface.create(
			"includeSurface", "surf", MFnNumericData::kBoolean, 1.0, &stat
		);
		FLUID_MAYA_CHECK(stat, "parameter creation");

		MFnNumericAttribute output_grid_offset;
		attr_output_grid_offset = output_grid_offset.create(
			"outputGridOffset", "goff", MFnNumericData::k3Int, 0.0, &stat
		);
		FLUID_MAYA_CHECK(stat, "parameter creation");
		FLUID_MAYA_CHECK_RETURN(output_grid_offset.setWritable(false), "parameter creation");
		FLUID_MAYA_CHECK_RETURN(output_grid_offset.setStorable(false), "parameter creation");

		MFnNumericAttribute output_grid_size;
		attr_output_grid_size = output_grid_size.create("outputGridSize", "gsz", MFnNumericData::k3Int, 0.0, &stat);
		FLUID_MAYA_CHECK(stat, "parameter creation");
		FLUID_MAYA_CHECK_RETURN(output_grid_size.setWritable(false), "parameter creation");
		FLUID_MAYA_CHECK_RETURN(output_grid_size.setStorable(false), "parameter creation");

		MFnTypedAttribute output_cells;
		attr_output_cells = output_cells.create("outputCells", "out", MFnData::kIntArray, MObject::kNullObj, &stat);
		FLUID_MAYA_CHECK(stat, "parameter creation");
		FLUID_MAYA_CHECK_RETURN(output_cells.setWritable(false), "parameter creation");
		FLUID_MAYA_CHECK_RETURN(output_cells.setStorable(false), "parameter creation");

		MFnTypedAttribute output_cells_ref;
		attr_output_cells_ref = output_cells_ref.create(
			"outputReferenceCells", "outr", MFnData::kIntArray, MObject::kNullObj, &stat
		);
		FLUID_MAYA_CHECK(stat, "parameter creation");
		FLUID_MAYA_CHECK_RETURN(output_cells_ref.setWritable(false), "parameter creation");
		FLUID_MAYA_CHECK_RETURN(output_cells_ref.setStorable(false), "parameter creation");


		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_input_mesh), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_cell_size), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_ref_grid_offset), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_ref_grid_size), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_include_interior), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_include_surface), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_output_grid_offset), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_output_grid_size), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_output_cells), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_output_cells_ref), "parameter registration");


		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_input_mesh, attr_output_grid_offset), "parameter registration"
		);
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_input_mesh, attr_output_grid_size), "parameter registration"
		);
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_input_mesh, attr_output_cells), "parameter registration"
		);
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_input_mesh, attr_output_cells_ref), "parameter registration"
		);

		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_cell_size, attr_output_grid_offset), "parameter registration"
		);
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_cell_size, attr_output_grid_size), "parameter registration"
		);
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_cell_size, attr_output_cells), "parameter registration"
		);
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_cell_size, attr_output_cells_ref), "parameter registration"
		);

		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_ref_grid_offset, attr_output_grid_offset), "parameter registration"
		);
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_ref_grid_offset, attr_output_grid_size), "parameter registration"
		);
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_ref_grid_offset, attr_output_cells), "parameter registration"
		);
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_ref_grid_offset, attr_output_cells_ref), "parameter registration"
		);

		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_ref_grid_size, attr_output_grid_offset), "parameter registration"
		);
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_ref_grid_size, attr_output_grid_size), "parameter registration"
		);
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_ref_grid_size, attr_output_cells), "parameter registration"
		);
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_ref_grid_size, attr_output_cells_ref), "parameter registration"
		);

		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_include_interior, attr_output_grid_offset), "parameter registration"
		);
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_include_interior, attr_output_grid_size), "parameter registration"
		);
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_include_interior, attr_output_cells), "parameter registration"
		);
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_include_interior, attr_output_cells_ref), "parameter registration"
		);

		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_include_surface, attr_output_grid_offset), "parameter registration"
		);
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_include_surface, attr_output_grid_size), "parameter registration"
		);
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_include_surface, attr_output_cells), "parameter registration"
		);
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_include_surface, attr_output_cells_ref), "parameter registration"
		);

		return MStatus::kSuccess;
	}

	MStatus voxelizer_node::compute(const MPlug &plug, MDataBlock &data_block) {
		if (
			plug != attr_output_grid_offset &&
			plug != attr_output_grid_size &&
			plug != attr_output_cells &&
			plug != attr_output_cells_ref
		) {
			return MStatus::kInvalidParameter;
		}

		MStatus stat;

		MDataHandle input_mesh_data = data_block.inputValue(attr_input_mesh, &stat);
		FLUID_MAYA_CHECK(stat, "retrieve attribute");
		MDataHandle cell_size_data = data_block.inputValue(attr_cell_size, &stat);
		FLUID_MAYA_CHECK(stat, "retrieve attribute");
		MDataHandle ref_grid_offset_data = data_block.inputValue(attr_ref_grid_offset, &stat);
		FLUID_MAYA_CHECK(stat, "retrieve attribute");
		MDataHandle ref_grid_size_data = data_block.inputValue(attr_ref_grid_size, &stat);
		FLUID_MAYA_CHECK(stat, "retrieve attribute");
		MDataHandle include_interior_data = data_block.inputValue(attr_include_interior, &stat);
		FLUID_MAYA_CHECK(stat, "retrieve attribute");
		MDataHandle include_surface_data = data_block.inputValue(attr_include_surface, &stat);
		FLUID_MAYA_CHECK(stat, "retrieve attribute");
		MDataHandle output_grid_offset_data = data_block.outputValue(attr_output_grid_offset, &stat);
		FLUID_MAYA_CHECK(stat, "retrieve attribute");
		MDataHandle output_grid_size_data = data_block.outputValue(attr_output_grid_size, &stat);
		FLUID_MAYA_CHECK(stat, "retrieve attribute");
		MDataHandle output_cells_data = data_block.outputValue(attr_output_cells, &stat);
		FLUID_MAYA_CHECK(stat, "retrieve attribute");
		MDataHandle output_cells_ref_data = data_block.outputValue(attr_output_cells_ref, &stat);
		FLUID_MAYA_CHECK(stat, "retrieve attribute");

		MFnMesh input_mesh(input_mesh_data.data(), &stat);
		FLUID_MAYA_CHECK(stat, "retrieve attribute");
		double cell_size = cell_size_data.asDouble();
		const double3 &ref_grid_offset = ref_grid_offset_data.asDouble3();
		const int3 &ref_grid_size = ref_grid_size_data.asInt3();
		bool include_interior = include_interior_data.asBool(), include_surface = include_surface_data.asBool();

		mesh<double, int, double, double, vec3d> vox_mesh;
		{ // vertices
			std::size_t num_verts = static_cast<std::size_t>(input_mesh.numVertices(&stat));
			FLUID_MAYA_CHECK(stat, "retrieve mesh");
			vox_mesh.positions.resize(num_verts);
			for (std::size_t i = 0; i < num_verts; ++i) {
				MPoint pt;
				FLUID_MAYA_CHECK_RETURN(
					input_mesh.getPoint(static_cast<int>(i), pt, MSpace::kWorld), "retrieve mesh"
				);
				vox_mesh.positions[i] = vec3d(pt.x, pt.y, pt.z);
			}
		}
		{ // indices
			MIntArray tri_counts, tri_ids;
			FLUID_MAYA_CHECK_RETURN(input_mesh.getTriangles(tri_counts, tri_ids), "retrieve mesh");
			std::size_t vert_count = static_cast<std::size_t>(tri_ids.length());
			vox_mesh.indices.resize(vert_count);
			for (std::size_t i = 0; i < vert_count; ++i) {
				vox_mesh.indices[i] = tri_ids[static_cast<unsigned int>(i)];
			}
		}

		// actual voxelization
		auto [bound_min, bound_max] = voxelizer::get_bounding_box(
			vox_mesh.positions.begin(), vox_mesh.positions.end()
		);
		voxelizer vox;
		vec3i grid_offset = vox.resize_reposition_grid_constrained(
			bound_min, bound_max, cell_size,
			vec3d(ref_grid_offset[0], ref_grid_offset[1], ref_grid_offset[2]),
			vec3s(
				static_cast<std::size_t>(ref_grid_size[0]),
				static_cast<std::size_t>(ref_grid_size[1]),
				static_cast<std::size_t>(ref_grid_size[2])
			)
		);
		vox.voxelize_mesh_surface(vox_mesh);
		vox.mark_exterior();

		// set output data
		{ // grid offset
			int3 &output_grid_offset = output_grid_offset_data.asInt3();
			output_grid_offset[0] = grid_offset.x;
			output_grid_offset[1] = grid_offset.y;
			output_grid_offset[2] = grid_offset.z;
			output_grid_offset_data.setClean();
		}
		{ // grid size
			int3 &output_grid_size = output_grid_size_data.asInt3();
			output_grid_size[0] = static_cast<int>(vox.voxels.get_size().x);
			output_grid_size[1] = static_cast<int>(vox.voxels.get_size().y);
			output_grid_size[2] = static_cast<int>(vox.voxels.get_size().z);
			output_grid_size_data.setClean();
		}
		// gather occupied cells
		std::vector<vec3s> occupied_cells;
		vox.voxels.for_each(
			[&occupied_cells, include_interior, include_surface](vec3s pos, voxelizer::cell_type type) {
				switch (type) {
				case voxelizer::cell_type::interior:
					if (include_interior) {
						occupied_cells.emplace_back(pos);
					}
					break;
				case voxelizer::cell_type::surface:
					if (include_surface) {
						occupied_cells.emplace_back(pos);
					}
					break;
				default:
					break;
				}
			}
		);
		{ // cells
			MIntArray array;
			array.setLength(static_cast<unsigned int>(occupied_cells.size() * 3));
			unsigned int i = 0;
			for (vec3s v : occupied_cells) {
				array[i++] = static_cast<int>(v.x);
				array[i++] = static_cast<int>(v.y);
				array[i++] = static_cast<int>(v.z);
			}

			MFnIntArrayData cells_array;
			MObject cells_array_data = cells_array.create(array, &stat);
			FLUID_MAYA_CHECK(stat, "finalize compute");
			FLUID_MAYA_CHECK_RETURN(output_cells_data.set(cells_array_data), "finalize compute");
			output_cells_data.setClean();
		}
		{ // ref cells
			MIntArray array;
			for (vec3s v : occupied_cells) {
				vec3i vref = vec3i(v) + grid_offset;
				if (
					vref.x >= 0 && vref.x < ref_grid_size[0] &&
					vref.y >= 0 && vref.y < ref_grid_size[1] &&
					vref.z >= 0 && vref.z < ref_grid_size[2]
				) {
					array.append(vref.x);
					array.append(vref.y);
					array.append(vref.z);
				}
			}

			MFnIntArrayData cells_array;
			MObject cells_array_data = cells_array.create(array, &stat);
			FLUID_MAYA_CHECK(stat, "finalize compute");
			FLUID_MAYA_CHECK_RETURN(output_cells_ref_data.set(cells_array_data), "finalize compute");
			output_cells_ref_data.setClean();
		}

		return MStatus::kSuccess;
	}
}
