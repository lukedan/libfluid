cmake_minimum_required(VERSION 3.8)
project(libfluid)

set(FLUID_BUILD_TESTBED ON CACHE BOOL "Whether or not to build the testbed.")


add_library(fluid)

target_compile_features(fluid
	PUBLIC cxx_std_17)
target_include_directories(fluid
	PUBLIC
		3rdparty/pcg-cpp/include
	INTERFACE src/)
target_sources(fluid
	PRIVATE
		src/data_structures/fluid_grid.cpp
		src/mesher.cpp
		src/pressure_solver.cpp
		src/simulation.cpp)


if(FLUID_BUILD_TESTBED)
	add_executable(testbed)

	find_package(OpenGL REQUIRED)
	find_package(glfw3 CONFIG REQUIRED)

	target_compile_features(testbed
		PRIVATE cxx_std_17)
	target_link_libraries(testbed
		PRIVATE fluid glfw OpenGL::GLU OpenGL::GL)
	target_sources(testbed
		PRIVATE src/main.cpp)
endif()
