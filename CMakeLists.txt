cmake_minimum_required(VERSION 3.9)
project(proto)

add_library(proto src/core/bvh.cpp)
target_include_directories(proto PUBLIC include)
set_target_properties(proto PROPERTIES CXX_STANDARD 17)
