#pragma once
#include <vector>
#include <array>
#include <cmath>
#include <Eigen/Dense>

namespace utils {

// -------------------------
// Helper: wrap angle
// -------------------------
inline double wrap_rad(double angle) {
    while (angle > M_PI)  angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

inline double wrap_deg(double angle_deg) {
    while (angle_deg > 180.0)  angle_deg -= 360.0;
    while (angle_deg < -180.0) angle_deg += 360.0;
    return angle_deg;
}


struct Point {
    double x;       
    double y;
    double index;       // angle of laserscan (deg)
    double r;           // r distance of laserscan

    Point(double x_, double y_, double index_, double r_)
        : x(x_), y(y_), index(index_), r(r_) {}
};


struct Pose {
    double x;
    double y;
    double theta;       // heading of robot

    Pose() : x(0), y(0), theta(0) {}
    Pose(double x_, double y_, double theta_)
        : x(x_), y(y_), theta(theta_) {}

    // Equivalent to Python __sub__
    Pose operator-(const Pose& old) const {
        return Pose(
            x - old.x,
            y - old.y,
            wrap_deg(theta - old.theta)
        );
    }
};




struct Node; // forward declaration - Node is referenced in edge

struct Edge {
    Node* parent;                   // pointer to parent node
    Eigen::Matrix3d transform;      // transform matrix(3×3)

    Edge(Node* parent, const Eigen::Matrix3d& T)
        : parent(parent), transform(T) {}
};

struct Node {
    int node_id;                                     // for debugging
    Pose pose;                                       // pose of  (x,y,theta)
    std::vector<std::unique_ptr<utils::Edge>> edges; // smart pointer for edges

    Node(int id, const Pose& p)
        : node_id(id), pose(p) {}

    void addEdge(Node* parent, const Eigen::Matrix3d& T) {
        /*
        * Add a new pointer to an edge 
        * 
        * @param parent - pointer to parent node 
        * @param T - transformation matrix from parent node
        */
       
        // EX: nodeA->addEdge(nodeB, transform_AB)
        edges.push_back(std::make_unique<utils::Edge>(parent, T));
    }
};

} 
