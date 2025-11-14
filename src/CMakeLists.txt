cmake_minimum_required(VERSION 3.10)
project(pose_graph_optimization_for_SLAM LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

add_executable(test_structs test_structs.cpp)

# Include directory path
set(INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include")

# Add Eigen include directory
target_include_directories(test_structs
    PUBLIC
        ${INCLUDE_DIR}
        /usr/include/eigen3
)
