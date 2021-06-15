cmake_minimum_required(VERSION 3.9)
project(proto LANGUAGES CXX VERSION 1.0)

add_library(proto INTERFACE)
target_include_directories(proto INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Make sure to only build tests when building this project,
# and not when importing it into another one.
if (CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    include(CTest)
    if (BUILD_TESTING)
        add_subdirectory(test)
    endif ()
endif ()

include(CMakePackageConfigHelpers)

# Allow using this library by simply calling `add_subdirectory(/path/to/proto)`
add_library(proto::proto ALIAS proto)

# Allow using this library by locating the config file from the build directory
export(TARGETS proto NAMESPACE proto:: FILE proto-config.cmake)
write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/proto-config-version.cmake COMPATIBILITY SameMajorVersion)

# Allow using this library by installing it
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/proto-config-version.cmake DESTINATION lib/cmake)
install(DIRECTORY include DESTINATION .)
install(TARGETS proto EXPORT proto-config)
install(EXPORT proto-config NAMESPACE proto:: DESTINATION lib/cmake)
