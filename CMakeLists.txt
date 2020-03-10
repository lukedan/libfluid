cmake_minimum_required(VERSION 3.8)
project(libfluid)


add_library(fluid)

target_compile_features(fluid
	PUBLIC cxx_std_17)
target_include_directories(fluid
	INTERFACE src/)
target_sources(fluid
	PRIVATE
		src/pressure_solver.cpp)


add_executable(testbed)

find_package(OpenGL REQUIRED)
find_package(glfw3 CONFIG REQUIRED)

target_compile_features(testbed
	PRIVATE cxx_std_17)
target_link_libraries(testbed
	PRIVATE fluid glfw OpenGL::GL)
target_sources(testbed
	PRIVATE src/main.cpp)
