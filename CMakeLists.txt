cmake_minimum_required(VERSION 3.8)
project(libfluid)

add_executable(testbed)

target_compile_features(testbed
	PRIVATE cxx_std_17)
target_sources(testbed
	PRIVATE
		src/math/vec.h
		src/main.cpp)
