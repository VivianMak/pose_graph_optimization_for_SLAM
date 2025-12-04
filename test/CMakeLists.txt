cmake_minimum_required(VERSION 3.10)
project(pose_graph_optimization_for_SLAM LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED NO_MODULE)


######## TESTS ###########################
# # Library: test_structs
add_library(test_structs
    src/test_structs.cpp
)
target_include_directories(test_structs PUBLIC include)
target_link_libraries(test_structs Eigen3::Eigen)
add_executable(test test_structs.cpp)