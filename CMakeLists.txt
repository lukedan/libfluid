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
		src/fluid_grid.cpp
		src/mesher.cpp
		src/pressure_solver.cpp
		src/simulation.cpp)

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
	target_link_libraries(fluid
		PUBLIC OpenMP::OpenMP_CXX)
endif()


if(FLUID_BUILD_TESTBED)
	add_executable(testbed)

	target_compile_features(testbed
		PRIVATE cxx_std_17)
	target_sources(testbed
		PRIVATE src/main.cpp)

	find_package(OpenGL REQUIRED)
	find_package(glfw3 CONFIG REQUIRED)
	target_link_libraries(testbed
		PRIVATE fluid glfw OpenGL::GLU OpenGL::GL)
endif()
