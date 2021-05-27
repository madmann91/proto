cmake_minimum_required(VERSION 3.9)
project(proto CXX)

add_library(proto INTERFACE)
target_include_directories(proto INTERFACE include)

# Make sure to only build tests when building this project,
# and not when importing it into another one.
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
    if (BUILD_TESTING)
        add_subdirectory(test)
    endif ()
endif ()
