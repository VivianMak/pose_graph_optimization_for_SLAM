#include "pose_graph.hpp"
#include "test_structs.hpp"
#include "utils.hpp"
#include "gn_optimizer.hpp"
#include "icp.hpp"
#include "viz.hpp"

#include "Eigen/Dense"

int main() {

    // READING SIM DATA
    std::cout << "Reading robot data ..." << std::endl;
    auto [scans, odoms] = loadData("../../data/robot_data.bin");
    std::cout << "number of odoms: " << odoms.size() << "\n";

    // testStructs();

    // POSE GRAPH
    //
    // The simulator's odometry is essentially ground truth, so we inject
    // compounding drift to give the optimizer something real to correct. The
    // LIDAR scans are untouched, which is the whole point: ICP still measures
    // the true relative motion, so those constraints can pull the drifted
    // trajectory back toward truth.
    pose_graph::NoiseConfig noise;
    noise.enabled     = true;
    noise.sigma_xy    = 0.010;  // per-sample translation error (m)
    noise.sigma_theta = 0.008;  // per-sample heading error (rad)
    noise.seed        = 42;      // fixed for reproducible runs

    pose_graph::PoseGraph graph;
    graph.build(scans, odoms, noise);

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

    // STEP_SIZE is from pose_graph.cpp.
    //
    // Node m is built from odom sample m*STEP_SIZE, so the scan that belongs to
    // node m is scans[m*STEP_SIZE]. Starting from scans.size()-1 instead walks a
    // grid offset by (scans.size()-1) % STEP_SIZE == 36, which pairs every ICP
    // constraint with scans taken 36 samples away from the nodes it constrains.
    // That is invisible on the straight runs and badly wrong through the turns.
    //
    // The bound is >= so the final edge (scan STEP_SIZE -> scan 0) is built too.
    // With > the first node gets no constraint at all and drifts free of the graph.
    for (size_t reading_idx = (num_nodes - 1) * STEP_SIZE; reading_idx >= STEP_SIZE; reading_idx -= STEP_SIZE) {
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

        // VISUALIZATION
        // Dump pre- and post-optimization routes so they can be compared.
        // getX() is stored reversed internally, so sort by node_id to line
        // it back up with the pre-optimized route before overlaying.
        auto pre_poses  = viz::posesFromNodes(nodes);
        auto opt_poses  = viz::posesFromNodes(resultX, /*sort_by_id=*/true);

        viz::writeCSV("../../data/pre_optimized.csv", pre_poses);
        viz::writeCSV("../../data/optimized.csv", opt_poses);

        // Raw odometry stream, drawn first so it sits behind the two routes.
        // This never passes through the pose graph -- it is the full ~4000
        // sample stream the 42 nodes were subsampled from, so it shows what the
        // graph is actually built on top of.
        auto odom_poses = viz::readCSV("../../data/noisy_odom_data.csv");

        viz::writeSVG("../../visualization/trajectories.svg", {
            {"odom (ground truth)", "#9aa0a6", odom_poses, viz::Style::Points},
            {"pre-optimized (drifted)", "crimson", pre_poses},
            {"optimized", "royalblue", opt_poses}
        });
    }


    return 0;
}
