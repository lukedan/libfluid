# libfluid

PIC simulation library.


## Buliding libfluid

libfluid uses CMake and requires a C++17-compliant compiler. Building has been tested on Visual Studio 2019. The library itself does not require any extra dependencies.

The repository also includes a testbed, which is built depending on the value of `FLUID_BUILD_TESTBED` (`ON` by default). The testbed requires `GLFW`.


## Acknowledgements

- [`rlguy/GridFluidSim3D`](https://github.com/rlguy/GridFluidSim3D) helped a lot for the solver implementation (although it does not seem to be strictly a PIC simulation as it claims).
- [`nepluno/apic2d`](https://github.com/nepluno/apic2d) is a good reference of the general algorithm.
