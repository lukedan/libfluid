#include "mesher_node.h"

/// \file
/// Implementation of the grid node.

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MPointArray.h>
#include <maya/MFnPointArrayData.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>

#include <fluid/mesher.h>
#include <fluid/data_structures/mesh.h>

#include "misc.h"

namespace fluid::maya {
	MObject
		mesher_node::attr_cell_size,
		mesher_node::attr_grid_size,
		mesher_node::attr_particles,
		mesher_node::attr_particle_offset,
		mesher_node::attr_particle_size,
		mesher_node::attr_particle_extents,
		mesher_node::attr_particle_check_radius,
		mesher_node::attr_output_mesh;
	MTypeId mesher_node::id{ 0x98765432 };

	void *mesher_node::creator() {
		return new mesher_node();
	}

	MStatus mesher_node::initialize() {
		MStatus stat;

		MFnNumericAttribute cell_size;
		attr_cell_size = cell_size.create("cellSize", "cell", MFnNumericData::kDouble, 1.0, &stat);
		FLUID_MAYA_CHECK(stat, "parameter creation");

		MFnNumericAttribute grid_size;
		attr_grid_size = grid_size.create("gridSize", "grid", MFnNumericData::k3Int, 50.0, &stat);
		FLUID_MAYA_CHECK(stat, "parameter creation");

		MFnTypedAttribute particles;
		attr_particles = particles.create("particles", "p", MFnData::kPointArray, MObject::kNullObj, &stat);
		FLUID_MAYA_CHECK(stat, "parameter creation");

		MFnNumericAttribute particle_offset;
		attr_particle_offset = particle_offset.create("particleOffset", "poff", MFnNumericData::k3Double, 0.0, &stat);
		FLUID_MAYA_CHECK(stat, "parameter creation");

		MFnNumericAttribute particle_size;
		attr_particle_size = particle_size.create("particleSize", "ps", MFnNumericData::kDouble, 0.5, &stat);
		FLUID_MAYA_CHECK(stat, "parameter creation");

		MFnNumericAttribute particle_extents;
		attr_particle_extents = particle_extents.create(
			"particleExtents", "pe", MFnNumericData::kDouble, 2.0, &stat
		);
		FLUID_MAYA_CHECK(stat, "parameter creation");

		MFnNumericAttribute particle_check_radius;
		attr_particle_check_radius = particle_check_radius.create(
			"particleCheckRadius", "pcr", MFnNumericData::kInt, 3, &stat
		);
		FLUID_MAYA_CHECK(stat, "parameter creation");

		MFnTypedAttribute output_mesh;
		attr_output_mesh = output_mesh.create("outputMesh", "out", MFnData::kMesh, MObject::kNullObj, &stat);
		FLUID_MAYA_CHECK(stat, "parameter creation");
		FLUID_MAYA_CHECK_RETURN(output_mesh.setWritable(false), "parameter creation");
		FLUID_MAYA_CHECK_RETURN(output_mesh.setStorable(false), "parameter creation");

		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_cell_size), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_grid_size), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_particles), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_particle_offset), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_particle_size), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_particle_extents), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_particle_check_radius), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(addAttribute(attr_output_mesh), "parameter registration");

		FLUID_MAYA_CHECK_RETURN(attributeAffects(attr_cell_size, attr_output_mesh), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(attributeAffects(attr_grid_size, attr_output_mesh), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(attributeAffects(attr_particles, attr_output_mesh), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(attributeAffects(attr_particle_offset, attr_output_mesh), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(attributeAffects(attr_particle_size, attr_output_mesh), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(attributeAffects(attr_particle_extents, attr_output_mesh), "parameter registration");
		FLUID_MAYA_CHECK_RETURN(
			attributeAffects(attr_particle_check_radius, attr_output_mesh), "parameter registration"
		);

		return MStatus::kSuccess;
	}

	MStatus mesher_node::compute(const MPlug &plug, MDataBlock &data_block) {
		if (plug == attr_output_mesh) {
			MStatus stat;

			MDataHandle cell_size_data = data_block.inputValue(attr_cell_size, &stat);
			FLUID_MAYA_CHECK(stat, "retrieve attribute");
			MDataHandle grid_size_data = data_block.inputValue(attr_grid_size, &stat);
			FLUID_MAYA_CHECK(stat, "retrieve attribute");
			MDataHandle particles_data = data_block.inputValue(attr_particles, &stat);
			FLUID_MAYA_CHECK(stat, "retrieve attribute");
			MDataHandle particle_offset_data = data_block.inputValue(attr_particle_offset, &stat);
			FLUID_MAYA_CHECK(stat, "retrieve attribute");
			MDataHandle particle_size_data = data_block.inputValue(attr_particle_size, &stat);
			FLUID_MAYA_CHECK(stat, "retrieve attribute");
			MDataHandle particle_extents_data = data_block.inputValue(attr_particle_extents, &stat);
			FLUID_MAYA_CHECK(stat, "retrieve attribute");
			MDataHandle particle_check_radius_data = data_block.inputValue(attr_particle_check_radius, &stat);
			FLUID_MAYA_CHECK(stat, "retrieve attribute");
			MDataHandle output_mesh_data = data_block.outputValue(attr_output_mesh, &stat);
			FLUID_MAYA_CHECK(stat, "retrieve attribute");

			// set mesher parameters
			mesher mesh_generator;
			// cell size
			mesh_generator.cell_size = cell_size_data.asDouble();
			// grid size
			const int3 &grid_size = grid_size_data.asInt3();
			for (std::size_t i = 0; i < 3; ++i) {
				if (grid_size[i] <= 0) {
					return MStatus::kInvalidParameter;
				}
			}
			mesh_generator.resize(vec3s(vec3i(grid_size[0], grid_size[1], grid_size[2])));
			// particle offset
			const double3 &particle_offset = particle_offset_data.asDouble3();
			mesh_generator.grid_offset = -vec3d(particle_offset[0], particle_offset[1], particle_offset[2]);
			// particle extent
			mesh_generator.particle_extent = particle_extents_data.asDouble();
			// particle check radius
			int check_radius = particle_check_radius_data.asInt();
			if (check_radius < 0) {
				return MStatus::kInvalidParameter;
			}
			mesh_generator.cell_radius = static_cast<std::size_t>(check_radius);

			// collect particles
			MFnPointArrayData particles_array(particles_data.data(), &stat);
			FLUID_MAYA_CHECK(stat, "particle retrieval");
			MPointArray particles = particles_array.array(&stat);
			FLUID_MAYA_CHECK(stat, "particle retrieval");
			unsigned num_particles = particles.length();
			std::vector<vec3d> particle_positions(num_particles);
			for (unsigned i = 0; i < num_particles; ++i) {
				const MPoint &mp = particles[i];
				particle_positions[i] = vec3d(mp.x, mp.y, mp.z);
			}

			// create mesh
			mesher::mesh_t m = mesh_generator.generate_mesh(particle_positions, particle_size_data.asDouble());
			MPointArray points;
			MIntArray face_counts, face_connects;
			for (vec3d p : m.positions) {
				FLUID_MAYA_CHECK_RETURN(points.append(MPoint(p.x, p.y, p.z)), "mesh creation");
			}
			for (std::size_t i = 0; i + 2 < m.indices.size(); i += 3) {
				FLUID_MAYA_CHECK_RETURN(face_counts.append(3), "mesh creation");
				FLUID_MAYA_CHECK_RETURN(face_connects.append(static_cast<int>(m.indices[i + 2])), "mesh creation");
				FLUID_MAYA_CHECK_RETURN(face_connects.append(static_cast<int>(m.indices[i + 1])), "mesh creation");
				FLUID_MAYA_CHECK_RETURN(face_connects.append(static_cast<int>(m.indices[i])), "mesh creation");
			}

			MFnMeshData output_mesh;
			MObject output_mesh_value = output_mesh.create(&stat);
			FLUID_MAYA_CHECK(stat, "mesh creation");
			MFnMesh mesh_fn;
			MObject output_mesh_object = mesh_fn.create(
				points.length(), face_counts.length(), points, face_counts, face_connects, output_mesh_value, &stat
			);
			FLUID_MAYA_CHECK(stat, "mesh creation");

			FLUID_MAYA_CHECK_RETURN(output_mesh_data.set(output_mesh_value), "finalize compute");
			output_mesh_data.setClean();
			return MStatus::kSuccess;
		}
		return MStatus::kUnknownParameter;
	}
}
