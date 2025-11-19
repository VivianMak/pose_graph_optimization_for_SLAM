cmake_minimum_required(VERSION 3.10)
project(pose_graph_optimization_for_SLAM LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

# Include directory path
set(INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include")

# Add Eigen include directory
include_directories(
        ${INCLUDE_DIR}
        /usr/include/eigen3
)

# Build executable
add_executable(main
    src/main.cpp
    src/read_data.cpp
    src/test_structs.cpp
    src/pose_graph.cpp
)

# add_executable(test_structs src/test_structs.cpp)
# add_executable(read_data src/read_data.cpp)