#include <vector>
#include <iostream>
#include <array>
#include <string>

// Define a type alias for a 3x3 matrix of doubles for convenience
using Matrix3x3 = std::array<std::array<double, 3>, 3>;

struct Pose {
    double x;
    double y;
    double theta;
};

class Node {
public:
    // Pose
    float x;
    float y;
    float theta;

    // List of laser scan points (Cartesian coordinates)
    std::vector<double> distances;

    // Transformation matrix (3x3)
    Matrix3x3 transform_mat;

    // Parent node (previous in sequence)
    Node* parent;

    // Insert constructor
};

class Edge {
public:
    // Edge data
    int from_id;
    int to_id;
    float dx;
    float dy;
    float dtheta;


};

class PoseGraph {
public:
    std::vector<Node*> nodes;
    std::vector<Edge> edges;


};
