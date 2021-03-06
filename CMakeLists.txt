cmake_minimum_required(VERSION 3.9)
project(libfluid)

set(FLUID_USE_OPENMP ON CACHE BOOL "Whether or not to use OpenMP.")
set(FLUID_BUILD_RENDERER ON CACHE BOOL "Whether or not to build the renderer.")
set(FLUID_BUILD_TESTBED ON CACHE BOOL "Whether or not to build the testbed.")
set(FLUID_BUILD_MAYA_PLUGIN ON CACHE BOOL "Whether or not to build the Maya plugin.")
set(FLUID_MAYA_DEVKIT_PATH "" CACHE PATH "Path to the Maya devkit.")


include(CheckIPOSupported)
check_ipo_supported(RESULT FLUID_IPO_SUPPORTED OUTPUT FLUID_IPO_MESSAGE)
if(NOT FLUID_IPO_SUPPORTED)
	message(WARNING "IPO is not supported: ${FLUID_IPO_MESSAGE}")
endif()


add_library(fluid)

target_compile_features(fluid
	PUBLIC cxx_std_17)
target_include_directories(fluid
	PUBLIC
		3rdparty/pcg-cpp/include/
		include/)
target_sources(fluid
	PRIVATE
		"src/data_structures/point_cloud.cpp"
		"src/data_structures/obstacle.cpp"
		"src/math/intersection.cpp"
		"src/math/warping.cpp"
		"src/mac_grid.cpp"
		"src/mesher.cpp"
		"src/pressure_solver.cpp"
		"src/simulation.cpp"
		"src/voxelizer.cpp")
if(FLUID_BUILD_RENDERER)
	target_sources(fluid
		PRIVATE
			"src/renderer/aabb_tree.cpp"
			"src/renderer/bidirectional_path_tracer.cpp"
			"src/renderer/bsdf.cpp"
			"src/renderer/camera.cpp"
			"src/renderer/fresnel.cpp"
			"src/renderer/material.cpp"
			"src/renderer/path_tracer.cpp"
			"src/renderer/primitive.cpp"
			"src/renderer/scene.cpp")
endif()
if(MSVC)
	target_compile_options(fluid
		PUBLIC /arch:AVX2
		PRIVATE /W4)
elseif(CMAKE_COMPILER_IS_GNUCXX)
	target_link_libraries(fluid
		PUBLIC stdc++fs)
endif()
if(FLUID_IPO_SUPPORTED)
	set_property(TARGET fluid PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

if(FLUID_USE_OPENMP)
	find_package(OpenMP)
	if(OpenMP_CXX_FOUND)
		target_link_libraries(fluid
			PUBLIC OpenMP::OpenMP_CXX)
	endif()
endif()


if(FLUID_BUILD_TESTBED)
	add_executable(testbed)

	target_compile_features(testbed
		PRIVATE cxx_std_17)
	target_sources(testbed
		PRIVATE
			"testbed/test_scenes.cpp"
			"testbed/main.cpp")

	find_package(OpenGL REQUIRED)
	find_package(glfw3 CONFIG REQUIRED)
	target_link_libraries(testbed
		PRIVATE fluid glfw OpenGL::GLU OpenGL::GL)
	if(FLUID_IPO_SUPPORTED)
		set_property(TARGET testbed PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
	endif()
endif()


if(FLUID_BUILD_MAYA_PLUGIN)
	add_library(fluid_maya SHARED)
	set_target_properties(fluid_maya PROPERTIES SUFFIX .mll)

	target_compile_features(fluid_maya
		PRIVATE cxx_std_17)
	target_sources(fluid_maya
		PRIVATE
			"plugins/maya/commands/add_fluid_source.cpp"
			"plugins/maya/commands/add_obstacle.cpp"
			"plugins/maya/commands/create_simulation_grid.cpp"
			"plugins/maya/nodes/grid_manipulator_node.cpp"
			"plugins/maya/nodes/grid_node.cpp"
			"plugins/maya/nodes/mesher_node.cpp"
			"plugins/maya/nodes/point_cloud_loader_node.cpp"
			"plugins/maya/nodes/voxelizer_node.cpp"
			"plugins/maya/main.cpp")

	target_include_directories(fluid_maya
		PRIVATE
			"${FLUID_MAYA_DEVKIT_PATH}/include")
	target_link_directories(fluid_maya
		PRIVATE
			"${FLUID_MAYA_DEVKIT_PATH}/lib")
	target_link_libraries(fluid_maya
		PRIVATE
			fluid
			Foundation OpenMaya OpenMayaUI OpenMayaAnim OpenMayaFX OpenMayaRender Image)
	if(FLUID_IPO_SUPPORTED)
		set_property(TARGET fluid_maya PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
	endif()
endif()
