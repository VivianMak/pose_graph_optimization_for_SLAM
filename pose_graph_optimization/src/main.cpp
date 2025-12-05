#include "pose_graph.hpp"
#include "test_structs.hpp"
#include "utils.hpp"
#include "gn_optimizer.hpp"
#include "icp.hpp"

#include "Eigen/Dense"

int main() {

    // READING SIM DATA
    std::cout << "Reading robot data ..." << std::endl;
    auto [scans, odoms] = loadData("../../data/robot_data.bin");
    std::cout << "number of odoms: " << odoms.size() << "\n";

    // testStructs();

    // POSE GRAPH
    pose_graph::PoseGraph graph;
    graph.build(scans, odoms);

    std::cout << "\nBuilding pose graph ..." << std::endl;

    // Inspect the graph thru terminal
    const auto& nodes = graph.nodes();
    const auto& edges = graph.edgeIndices();

    std::cout << "Created nodes: " << nodes.size() << "\n";
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
    std::vector<std::unique_ptr<GN::Mat33>> Z_;

    size_t num_readings = scans.size(); 

    // Select ICP parameters
    size_t num_iterations = 100;
    size_t num_neighbors = 10;
    double error_weighting = 0.5;

    size_t num_nodes = nodes.size();

    // STEP_SIZE is from pose_graph.cpp
    for (size_t reading_idx = num_readings - 1; reading_idx > STEP_SIZE; reading_idx -= STEP_SIZE) {
        // Readings to find lidar transform between
        size_t src_idx = reading_idx;
        size_t dst_idx = reading_idx - STEP_SIZE;

        // Convert selected scans into matrices that ICP can be done on
        Eigen::MatrixXd src_point_matrix = scan_to_matrix(scans[src_idx]); // 3, n
        Eigen::MatrixXd dst_point_matrix = scan_to_matrix(scans[dst_idx]); // 3, n

        // Calculate the odom transform between the selected poses
        Eigen::Matrix3d odom_htm = htm_between_poses(
            nodes[num_nodes-2]->pose,
            nodes[num_nodes-1]->pose
        ); // 3, 3

        // // Run ICP
        Eigen::Matrix3d src_to_dst_htm = icp(src_point_matrix, dst_point_matrix, num_iterations, odom_htm, num_neighbors, error_weighting);

        Z_.push_back(std::make_unique<GN::Mat33>(src_to_dst_htm));
        num_nodes -= 1;
    }

    // GLOBAL OPTIMIZATION

    // set configurations of optimization
    GN::GN_Config config = {
        10,
        0.1,
        Eigen::Matrix3d::Identity()
    };

    GN::GnOptimizer gn(Z_, nodes, config);
    bool isOptimized = gn.gnOptimizer(); // should also return X 

    if(isOptimized){
        std::cout << "IS OPTIMIZED FINISHED\n";
        const auto& resultX = gn.getX();
        // const auto& n = resultX[4];
        std::cout << "The optimized nodes are: " << resultX.size() << std::endl;
        // Comparison of nodes before v after global optimization
        // for (size_t i = 0; i < N_.size(); i++) {
        //     double dx = N_[i].pose.x - X[i].pose.x;
        // }
    }
        

    return 0;
}
