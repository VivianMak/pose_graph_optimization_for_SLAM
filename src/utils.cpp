#include <vector>
#include <iostream>
#include <array>

// Define a type alias for a 3x3 matrix of doubles for convenience.
using Matrix3x3 = std::array<std::array<double, 3>, 3>;

class Node {
public:
    // pose
    float x;
    float y;
    float theta;
    // List of laserscan points in cartesian coordinates
    std::vector<double> distances;

    // Transformation matrix (3x3)
    Matrix3x3 transform_mat;

    // Parent node will always be previous node on the list
}

class Edge {
public:
    std::vector<
}