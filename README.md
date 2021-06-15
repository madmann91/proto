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

## License

```
Copyright 2021 Arsène Pérard-Gayot

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
