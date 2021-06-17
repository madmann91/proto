add_executable(proto_test main.cpp)
target_link_libraries(proto_test proto)
set_target_properties(proto_test PROPERTIES CXX_STANDARD 20)

add_test(NAME proto_test COMMAND proto_test)
