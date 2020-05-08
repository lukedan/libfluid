# libfluid

PIC/FLIP/APIC fluid simulation library with a Maya plugin.


## Buliding libfluid

libfluid uses CMake and requires a C++17-compliant compiler. Building has been tested on Visual Studio 2019. The library itself does not require any extra dependencies.

### The testbed

The repository also includes a testbed, which is built depending on the value of `FLUID_BUILD_TESTBED` (`ON` by default). The testbed requires `GLFW`.

### Maya plugin

The building of the Maya plugin is dependent on the variable `FLUID_BUILD_MAYA_PLUGIN`. The variable `FLUID_MAYA_DEVKIT_PATH` should be set to point to the Maya devkit directory that contains the `include` and `lib` directories.


## Acknowledgements

- [`rlguy/GridFluidSim3D`](https://github.com/rlguy/GridFluidSim3D) helped a lot for the solver implementation.
- [`nepluno/apic2d`](https://github.com/nepluno/apic2d) is a good reference of the general algorithm.
