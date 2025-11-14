cmake_minimum_required(VERSION 3.5)
project(pose_graph_exploration)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

set(DEPENDENCIES
"Eigen3"
)

include_directories(
    include
)

set(POSE_GRAPH_EXECUTABLE "pose_graph")

add_executable(${POSE_GRAPH_EXECUTABLE} src/pose_graph.cpp include/pose_graph_header.cpp)
target_link_libraries(${POSE_GRAPH_EXECUTABLE} PUBLIC Eigen3::Eigen)
ament_target_dependencies(${POSE_GRAPH_EXECUTABLE} ${DEPENDENCIES})

install(TARGETS
    ${POSE_GRAPH_EXECUTABLE}
    DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_package()

