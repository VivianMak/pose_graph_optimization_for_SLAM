#pragma once

#include <vector>
#include <memory>
#include <Eigen/Dense>

#include "utils.hpp"

namespace pose_graph {

// PoseGraph: nodes + edges (structure according to Vivian's .py skel code)
//
// Stores a simple pose graph with:
//   - owned Node instances
//   - non-owning pointers to edges (edges live in Node)
//   - basic loop-closure detection against the first node
//
// Assumes:
//   - utils::Pose::theta is expressed in radians.
//
class PoseGraph {
public:
    PoseGraph();

    // Add node to graph
    utils::Node* addNode(int node_id, const utils::Pose& pose);

    // Register edge globally
    void registerEdge(utils::Edge* edge_ptr);

    // Graph accessors
    const std::vector<std::unique_ptr<utils::Node>>& nodes() const;
    const std::vector<utils::Edge*>& edges() const;

    // Assign weight (placeholder, please check)
    void assignWeight();

    // Check loop closure
    void checkLoopClosure(utils::Node* node);

    // Loop closure status
    bool isLoopClosed() const;
    void resetLoopClosed();

private:
    // Helper: check loop thresholds
    bool withinThreshold(const utils::Pose& delta) const;

    // graph storage (owned nodes, non-owning edges)
    std::vector<std::unique_ptr<utils::Node>> nodes_;
    std::vector<utils::Edge*> edges_;

    // loop-closure thresholds (position in meters, heading in radians)
    double threshold_x_;
    double threshold_y_;
    double threshold_theta_rad_;

    // reference node for loop closure (first node added)
    utils::Node* check_node_;

    // loop-closure flag
    bool closed_loop_;
};

} 