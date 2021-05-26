cmake_minimum_required(VERSION 3.9)
project(proto CXX)

add_library(proto INTERFACE)
target_include_directories(proto INTERFACE include)
set_target_properties(proto PROPERTIES CXX_STANDARD 17)

# Enable IPO in Release mode
include(CheckIPOSupported)
check_ipo_supported(RESULT IPO_SUPPORTED)
if (IPO_SUPPORTED)
    set_target_properties(proto PROPERTIES INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
endif ()

# Make sure to only build tests when building this project,
# and not when importing it into another one.
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
    if (BUILD_TESTING)
        add_subdirectory(test)
    endif ()
endif ()
