cmake_minimum_required(VERSION 3.10)
project(pose_graph_optimization_for_SLAM LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)

find_package(Eigen3 REQUIRED NO_MODULE)


##### Build Libraries #####

# Library: read_data
add_library(read_data
    src/read_data.cpp
)
target_include_directories(read_data PUBLIC include)
target_link_libraries(read_data Eigen3::Eigen)

# Library: pose_graph
add_library(pose_graph
    src/pose_graph.cpp
)
target_include_directories(pose_graph PUBLIC include)
target_link_libraries(pose_graph Eigen3::Eigen)

# Library: icp
add_library(icp
    src/icp.cpp
)
target_include_directories(icp PUBLIC include)
target_link_libraries(icp Eigen3::Eigen)

# Library: test_structs
add_library(test_structs
    src/test_structs.cpp
)
target_include_directories(test_structs PUBLIC include)
target_link_libraries(test_structs Eigen3::Eigen)

# # Library: gn_optimizer
# add_library(gn_optimizer
#     src/gn_optimizer.cpp
# )
# target_include_directories(gn_optimizer PUBLIC include)
# target_link_libraries(gn_optimizer Eigen3::Eigen)

# Main program executable
add_executable(main src/main.cpp)
target_link_libraries(main
    read_data
    pose_graph
    icp
    test_structs
    # gn_optimizer
)
