#include <vector>
#include <iostream>
#include <array>

// Define a type alias for a 3x3 matrix of doubles for convenience
using Matrix3x3 = std::array<std::array<double, 3>, 3>;

class Node {
public:
    // Pose data
    float x;
    float y;
    float theta;

    // List of laser scan points (Cartesian coordinates)
    std::vector<double> distances;

    // Transformation matrix (3x3)
    Matrix3x3 transform_mat;

    // Parent node (previous in sequence)
    Node* parent;

    // Begin constructor
    Node(float x_, float y_, float theta_, Node* parent_ = nullptr)
    : x(x_), y(y_), theta(theta_), parent(parent_) {
    // Initialize transform as identity matrix
    transform_mat = {{{1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{0.0, 0.0, 1.0}}};
    }
};

class Edge {
public:
    // Edge data
    int from_id;
    int to_id;
    float dx;
    float dy;
    float dtheta;

    // Begin constructor
    Edge(int from, int to, float dx_, float dy_, float dtheta_)
        : from_id(from), to_id(to), dx(dx_), dy(dy_), dtheta(dtheta_) {}
};

class PoseGraph {
public:
    std::vector<Node*> nodes;
    std::vector<Edge> edges;


};
