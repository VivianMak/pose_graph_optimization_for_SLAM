#include "pose_graph.hpp"
#include "test_structs.hpp"
#include "utils.hpp"
#include "gn_optimizer.hpp"

#include "Eigen/Dense"

int main() {

    // READING SIM DATA
    std::cout << "Reading robot data ..." << std::endl;
    auto [scans, odoms] = loadData("../../data/robot_data.bin");

    // testStructs();

    // POSE GRAPH
    pose_graph::PoseGraph graph;
    graph.build(scans, odoms);

    std::cout << "\nBuilding pose graph ..." << std::endl;

    // Inspect the graph thru terminal
    const auto& fnodes = graph.nodes();
    const auto& edges = graph.edgeIndices();

    std::cout << "Created nodes: " << fnodes.size() << "\n";
    std::cout << "Created edges: " << edges.size() << "\n\n";

    // print first few nodes
    // for (size_t i = 0; i < std::min<size_t>(nodes.size(), 10); i++) {
    //     const auto& n = nodes[i];
    //     std::cout << "Node " << n->node_id
    //               << ": pose = (" << n->pose.x 
    //               << ", " << n->pose.y 
    //               << ", " << n->pose.theta << " rad)\n";
    // }
    // print some edges
    // std::cout << "\nEdges:\n";
    // for (size_t i = 0; i < std::min<size_t>(edges.size(), 5); i++) {
    //     std::cout << " Edge from node "
    //               << edges[i].parent_index 
    //               << " to " 
    //               << edges[i].child_index
    //               << "\n";
    // }



    // ICP



    // Fake Z_ test data (3 matrices)
    std::vector<std::unique_ptr<GN::Mat33>> Z_test;

    GN::Mat33 T1;
    T1 << 1, 0, 1,
        0, 1, 2,
        0, 0, 1;

    GN::Mat33 T2;
    T2 << cos(0.5), -sin(0.5), 0.3,
        sin(0.5),  cos(0.5), 0.7,
        0,         0,        1;

    GN::Mat33 T3;
    T3 << cos(-1.2), -sin(-1.2), -0.8,
        sin(-1.2),  cos(-1.2),  1.5,
        0,          0,          1;

    Z_test.push_back(std::make_unique<GN::Mat33>(T1));
    Z_test.push_back(std::make_unique<GN::Mat33>(T2));
    Z_test.push_back(std::make_unique<GN::Mat33>(T3));

    // Fake nodes test data (3 nodes)
    std::vector<std::unique_ptr<utils::Node>> nodes;

    // Assume Pose has a constructor Pose(x, y, theta)
    utils::Pose p1(0.0, 0.0, 0.0);
    utils::Pose p2(1.0, 2.0, 0.5);
    utils::Pose p3(-1.0, 1.5, -0.7);

    // Create nodes and push into vector
    nodes.push_back(std::make_unique<utils::Node>(0, p1));
    nodes.push_back(std::make_unique<utils::Node>(1, p2));
    nodes.push_back(std::make_unique<utils::Node>(2, p3));

    // Optionally, add edges if needed
    nodes[1]->addEdge(nodes[0].get(), *Z_test[0]);  // node1 -> node0
    nodes[2]->addEdge(nodes[1].get(), *Z_test[1]);  // node2 -> node1

    // GLOBAL OPTIMIZATION

    // set configurations of optimization
    GN::GN_Config config = {
        10,
        0.1,
        Eigen::Matrix3d::Identity()
    };

    GN::GnOptimizer gn(Z_test, nodes, config);
    bool isOptimized = gn.gnOptimizer(); // should also return X 

    if(isOptimized){
        std::cout << "IS OPTIMIZED FINISHED\n";
        const auto& resultX = gn.getX();
        // Comparison of nodes before v after global optimization
        // for (size_t i = 0; i < N_.size(); i++) {
        //     double dx = N_[i].pose.x - X[i].pose.x;
        // }
    }
        

    return 0;
}
