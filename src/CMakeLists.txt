cmake_minimum_required(VERSION 3.0)

project(TestStructs LANGUAGES CXX)

# Include directory path
set(INCLUDE_DIR "${CMAKE_SOURCE_DIR}/include")

# Create executable
add_executable(main main.cpp)

# Add include directory to the 'main' target
target_include_directories(main 
    PUBLIC 
        ${INCLUDE_DIR}
)
