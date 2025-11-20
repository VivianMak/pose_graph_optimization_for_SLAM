cmake_minimum_required(VERSION 3.5)
project(icp)

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

set(ICP_EXECUTABLE "icp")

add_executable(${ICP_EXECUTABLE} src/icp.cpp include/icp.cpp)
target_link_libraries(${ICP_EXECUTABLE} PUBLIC Eigen3::Eigen)
ament_target_dependencies(${ICP_EXECUTABLE} ${DEPENDENCIES})

install(TARGETS
    ${ICP_EXECUTABLE}
    DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_package()

