# Proto

`proto` is an utility library that I use in my own graphics-related projects.
It contains basic infrastructure to do small vector linear algebra, and some simple ray-tracing constructs.

## Installing

Since this project is a header-only library, it can be added to your project simply by
writing the following CMake code to your CMakeLists.txt:

    add_subdirectory(/path/to/proto)

This line will provide the target `proto::proto`, which you can link against.

Alternatively, if you have several projects that depend on proto, you can use a call to
`find_package(proto)`:

    find_package(proto REQUIRED)

This will require to set the CMake variable `proto_DIR`. For that, you can alternatively
use the build directory, or install the library somewhere, say `/path/to/install`, and
set `proto_DIR` to the path `/path/to/install/lib/cmake`.
