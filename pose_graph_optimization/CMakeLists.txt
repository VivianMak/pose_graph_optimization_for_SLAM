cmake_minimum_required(VERSION 3.16)
project(pose_graph_optimization_for_SLAM LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Emit compile_commands.json so clangd / .clang-tidy pick up our include paths
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

##### External: Eigen #####
#
# Eigen is header-only, so there is nothing to compile or link -- it only needs
# to be on the include path. We resolve it two ways so the project builds with
# or without a package manager:
#
#   1. A system install (Homebrew, MacPorts, apt, or `cmake --install` into
#      ~/.local). Configure with -DCMAKE_PREFIX_PATH=$HOME/.local if it lives
#      somewhere CMake does not search by default.
#   2. A vendored copy in third_party/, same as we already do for nanoflann.
#      Get one with:
#        git clone --depth 1 --branch 3.4.0 \
#            https://gitlab.com/libeigen/eigen.git /tmp/eigen-src
#        cp -r /tmp/eigen-src/Eigen third_party/
#
# Either way the libraries below just link `eigen` and stay agnostic.

add_library(eigen INTERFACE)

find_package(Eigen3 QUIET NO_MODULE)

# Places a vendored Eigen/ directory is allowed to live (repo root, then here)
set(EIGEN_VENDOR_CANDIDATES
    "${CMAKE_CURRENT_SOURCE_DIR}/../third_party"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party"
)

set(EIGEN_VENDOR_DIR "")
foreach(candidate IN LISTS EIGEN_VENDOR_CANDIDATES)
    if(EXISTS "${candidate}/Eigen/Dense")
        set(EIGEN_VENDOR_DIR "${candidate}")
        break()
    endif()
endforeach()

if(TARGET Eigen3::Eigen)
    message(STATUS "Eigen: system install at ${Eigen3_DIR}")
    target_link_libraries(eigen INTERFACE Eigen3::Eigen)
elseif(EIGEN_VENDOR_DIR)
    message(STATUS "Eigen: vendored copy at ${EIGEN_VENDOR_DIR}")
    # SYSTEM suppresses warnings from third-party headers in our build output
    target_include_directories(eigen SYSTEM INTERFACE "${EIGEN_VENDOR_DIR}")
else()
    message(FATAL_ERROR
        "Eigen not found.\n"
        "Install it (brew install eigen / port install eigen3), or vendor the\n"
        "headers -- from the repo root:\n"
        "  git clone --depth 1 --branch 3.4.0 "
        "https://gitlab.com/libeigen/eigen.git /tmp/eigen-src\n"
        "  cp -r /tmp/eigen-src/Eigen third_party/\n"
        "If Eigen is installed somewhere non-standard, point CMake at it with\n"
        "  cmake -DCMAKE_PREFIX_PATH=$ENV{HOME}/.local ..")
endif()

##### Build Libraries #####

# Library: read_data
add_library(read_data
    src/read_data.cpp
)
target_include_directories(read_data PUBLIC include)
target_link_libraries(read_data PUBLIC eigen)

# Library: pose_graph
add_library(pose_graph
    src/pose_graph.cpp
)
target_include_directories(pose_graph PUBLIC include)
target_link_libraries(pose_graph PUBLIC eigen)

# Library: icp
add_library(icp
    src/icp.cpp
)
target_include_directories(icp PUBLIC include)
target_link_libraries(icp PUBLIC eigen)


# Library: gn_optimizer
add_library(gn_optimizer
    src/gn_optimizer.cpp
)
target_include_directories(gn_optimizer PUBLIC include)
target_link_libraries(gn_optimizer PUBLIC eigen)



# Main program executable
add_executable(main src/main.cpp)
target_link_libraries(main
    read_data
    pose_graph
    icp
    # test_structs
    gn_optimizer
)
