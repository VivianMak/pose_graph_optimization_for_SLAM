#pragma once
#include <vector>
#include <array>
#include <cmath>

namespace slam {

// -------------------------
// Helper: wrap angle
// -------------------------
inline double wrap(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// -------------------------
// Point (immutable-style)
// -------------------------
struct Point {
    double x;
    double y;
    double index;
    double r;

    Point(double x_, double y_, double index_, double r_)
        : x(x_), y(y_), index(index_), r(r_) {}
};

// -------------------------
// Pose
// -------------------------
struct Pose {
    double x;
    double y;
    double theta;

    Pose() : x(0), y(0), theta(0) {}
    Pose(double x_, double y_, double theta_)
        : x(x_), y(y_), theta(theta_) {}

    // Equivalent to Python __sub__
    Pose operator-(const Pose& old) const {
        return Pose(
            x - old.x,
            y - old.y,
            wrap(theta - old.theta)
        );
    }
};

// -------------------------
// Node
// -------------------------
struct Node {
    int node_id;
    Pose pose;
    std::vector<std::vector<double>> scans;  
    // a vector of scan arrays, adjust type later as needed

    Node(int id, const Pose& p)
        : node_id(id), pose(p) {}
};

// -------------------------
// Edge
// -------------------------
struct Edge {
    int parent_id;
    int child_id;

    // 3x3 transform matrix (fixed-size array)
    std::array<std::array<double, 3>, 3> transform;

    Edge(int parent, int child,
         const std::array<std::array<double, 3>, 3>& T)
        : parent_id(parent), child_id(child), transform(T) {}
};

} // namespace slam
