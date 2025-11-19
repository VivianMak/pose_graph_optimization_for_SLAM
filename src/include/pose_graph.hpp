#pragma once
#include <iostream>
#include <vector>

#include "utils.hpp"

namespace pose_graph

class PoseGraph{
    public:
    std::vector node_list;
    utils::Pose threshold(0.5, 0.5, 30);

    // First node
    utils::Node check_node;
    bool closed_loop(false);

    void push_node(utils::Node);

    void check_loop_closure(utils::Node);

    bool is_closed_loop();
        // force an edge constraint btwn first node and last node

}