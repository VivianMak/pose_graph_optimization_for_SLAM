#include "pose_graph.hpp"

#include <iostream>
#include <cmath>

namespace pose_graph {

// Constructor
PoseGraph::PoseGraph()
    : threshold_x_(0.5),              // meters
      threshold_y_(0.5),              // meters
      threshold_theta_rad_(M_PI / 6), // 30 deg ≈ π/6 radians
      check_node_(nullptr),
      closed_loop_(false)
{}

// Add node to graph
utils::Node* PoseGraph::addNode(int node_id, const utils::Pose& pose) {
    std::cout << "---------- ADDING NODE --------------" << std::endl;

    nodes_.push_back(std::make_unique<utils::Node>(node_id, pose));
    utils::Node* node_ptr = nodes_.back().get();

    // only first node used as loop-closure reference (please check mo)
    if (!check_node_) {
        check_node_ = node_ptr;
    }

    return node_ptr;
}

// Register edge globally
void PoseGraph::registerEdge(utils::Edge* edge_ptr) {
    if (!edge_ptr) {
        return;
    }

    std::cout << "---------- ADDING EDGE --------------" << std::endl;
    edges_.push_back(edge_ptr);
}

// Graph accessors
const std::vector<std::unique_ptr<utils::Node>>& PoseGraph::nodes() const {
    return nodes_;
}

const std::vector<utils::Edge*>& PoseGraph::edges() const {
    return edges_;
}

// Assign weight (temporary placeholder, please check Vivian)
void PoseGraph::assignWeight() {
    // TODO: implement edge weighting / information matrices if needed
}

// Check loop closure
void PoseGraph::checkLoopClosure(utils::Node* node) {
    if (!check_node_ || !node) {
        return;
    }

    // pose difference wrt reference node
    utils::Pose delta = node->pose - check_node_->pose;

    if (withinThreshold(delta)) {
        std::cout << "------------- LOOP CLOSURE FOUND --------------" << std::endl;

        // heading already in radians
        double t = delta.theta;

        // relative SE2 transform (3×3 homogeneous)
        Eigen::Matrix3d T;
        T << std::cos(t), -std::sin(t), delta.x,
             std::sin(t),  std::cos(t), delta.y,
             0.0,          0.0,         1.0;

        // parent: reference node, child: current node
        check_node_->addEdge(node, T);

        // keep a raw pointer to the last created edge
        utils::Edge* new_edge = check_node_->edges.back().get();
        registerEdge(new_edge);

        closed_loop_ = true;
    }
}

// Loop closure status
bool PoseGraph::isLoopClosed() const {
    return closed_loop_;
}

void PoseGraph::resetLoopClosed() {
    closed_loop_ = false;
}

// Helper: threshold check
bool PoseGraph::withinThreshold(const utils::Pose& d) const {
    return (std::fabs(d.x)     <= threshold_x_) &&
           (std::fabs(d.y)     <= threshold_y_) &&
           (std::fabs(d.theta) <= threshold_theta_rad_);
}

}
