#include "pose_graph.hpp"
#include <iostream>
#include <cmath>

namespace pose_graph {

PoseGraph::PoseGraph()
    : threshold_x_(0.5),
      threshold_y_(0.5),
      threshold_theta_rad_(M_PI / 6.0),
      check_node_(nullptr),
      closed_loop_(false)
{
    // stupid ahh constructor outline
}

utils::Node* PoseGraph::addNode(int node_id, const utils::Pose& pose) {
    std::cout << "[PoseGraph] Adding node " << node_id << std::endl;

    nodes_.push_back(std::make_unique<utils::Node>(node_id, pose));
    utils::Node* node_ptr = nodes_.back().get();

    // first node becomes loop-closure reference
    if (!check_node_) {
        check_node_ = node_ptr;
    }

    return node_ptr;
}

void PoseGraph::checkLoopClosure(utils::Node* node) {
    std::cout << "[PoseGraph] Checking loop closure..." << std::endl;

    // BASIC OUTLINE ONLY, real logic added later
    // e.g. compute delta pose, compare thresholds, add edge dot dot dot

    // for outline demonstration:
    (void)node;  // suppresses unused thingie warning
}

bool PoseGraph::isLoopClosed() const {
    return closed_loop_;
}

void PoseGraph::resetLoopClosed() {
    closed_loop_ = false;
}

bool PoseGraph::withinThreshold(const utils::Pose& delta) const {
    // outline only real threshold checks will be released later
    (void)delta;
    return false;
}

}
