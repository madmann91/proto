cmake_minimum_required(VERSION 3.9)
project(proto)

add_library(proto INTERFACE)
target_include_directories(proto INTERFACE include)
set_target_properties(proto PROPERTIES CXX_STANDARD 17)

# Enable IPO in Release mode
include(CheckIPOSupported)
check_ipo_supported(RESULT IPO_SUPPORTED)
if (IPO_SUPPORTED)
    set_target_properties(proto PROPERTIES INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
endif ()

include(CTest)
if (BUILD_TESTING)
    add_subdirectory(test)
endif ()
