#include "icp.h"

int main() {
    // Select scans to find lidar transform between
    int scan_one_idx = 0;
    int scan_two_idx = 2695;

    // Read and save odom and lidar data
    auto noisy_poses = std::make_unique<std::vector<Pose>>();
    auto laser_scans = std::make_unique<std::vector<std::vector<LaserScan>>>();

    read_odom("noisy_odom_data.csv", *noisy_poses);
    read_lidar("lidar_scans.csv", *laser_scans);

    // Convert selected scans into matrices that ICP can be done on
    Eigen::MatrixXd src_point_matrix = scan_to_matrix(laser_scans->at(scan_two_idx)); // 3, n
    Eigen::MatrixXd dst_point_matrix = scan_to_matrix(laser_scans->at(scan_one_idx)); // 3, n

    // Calculate the odom transform between the selected poses
    Eigen::Matrix3d odom_htm = htm_between_poses(
        noisy_poses->at(scan_one_idx),
        noisy_poses->at(scan_two_idx)
    ); // 3, 3

    // Select ICP parameters
    size_t num_iterations = 100;
    size_t num_neighbors = 10;
    double error_weighting = 0.5;

    // Run ICP
    Eigen::Matrix3d src_to_dst_htm = icp(src_point_matrix, dst_point_matrix, num_iterations, odom_htm, num_neighbors, error_weighting);

    // Print the lidar htm
    std::cout << src_to_dst_htm << "\n";
}
