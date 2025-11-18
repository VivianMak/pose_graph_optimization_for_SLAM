#pragma once

#include <vector>
#include <memory>
#include "utils.hpp"

namespace pose_graph {

// -------------------------
// PoseGraph outline
// -------------------------
class PoseGraph {
public:
    PoseGraph();

    utils::Node* addNode(int node_id, const utils::Pose& pose);
    void checkLoopClosure(utils::Node* node);

    bool isLoopClosed() const;
    void resetLoopClosed();

private:
    // internal helpers
    bool withinThreshold(const utils::Pose& delta) const;

    // graph data
    std::vector<std::unique_ptr<utils::Node>> nodes_;
    std::vector<utils::Edge*> edges_;

    // loop closure parameters
    double threshold_x_;
    double threshold_y_;
    double threshold_theta_rad_;

    utils::Node* check_node_;   // reference node for loop closure
    bool closed_loop_;          // tracks loop-closure state
};

}
