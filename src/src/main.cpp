#include "pose_graph.hpp"
#include "utils.hpp"

int main() {
    std::cout << "Reading robot data...\n";
    auto [scans, odoms] = loadData("../../data/robot_data.bin");

    // Build the pose graph
    pose_graph::PoseGraph graph;
    graph.build(scans, odoms);

    std::cout << "Pose graph built.\n";

    // Inspect the graph thru terminal
    const auto& nodes = graph.nodes();
    const auto& edges = graph.edgeIndices();

    std::cout << "Total nodes: " << nodes.size() << "\n";
    std::cout << "Total edges: " << edges.size() << "\n\n";

    // print first few nodes
    for (size_t i = 0; i < std::min<size_t>(nodes.size(), 5); i++) {
        const auto& n = nodes[i];
        std::cout << "Node " << n->node_id
                  << ": pose = (" << n->pose.x 
                  << ", " << n->pose.y 
                  << ", " << n->pose.theta << " deg)\n";
    }

    // print some edges
    std::cout << "\nEdges:\n";
    for (size_t i = 0; i < std::min<size_t>(edges.size(), 5); i++) {
        std::cout << " Edge from node "
                  << edges[i].parent_index 
                  << " to " 
                  << edges[i].child_index
                  << "\n";
    }

    return 0;
}
