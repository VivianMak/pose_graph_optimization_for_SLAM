#include "icp.h"

int main() {
    int scan_one_idx = 0;
    int scan_two_idx = 2695;

    auto noisy_poses = std::make_unique<std::vector<Pose>>();

    read_odom("noisy_odom_data.csv", *noisy_poses);

    auto laser_scans = std::make_unique<std::vector<std::vector<LaserScan>>>();
    
    read_lidar("lidar_scans.csv", *laser_scans);

    Eigen::MatrixXd src_point_matrix = scan_to_matrix(laser_scans->at(scan_two_idx)); // 3, n
    Eigen::MatrixXd dst_point_matrix = scan_to_matrix(laser_scans->at(scan_one_idx)); // 3, n

    Eigen::Matrix3d odom_htm = htm_between_poses(
        noisy_poses->at(scan_one_idx),
        noisy_poses->at(scan_two_idx)
    ); // 3, 3

    size_t num_iterations = 100;
    size_t num_neighbors = 10;
    double error_weighting = 0.5;

    Eigen::Matrix3d src_to_dst_htm = icp(src_point_matrix, dst_point_matrix, num_iterations, odom_htm, num_neighbors, error_weighting);

    std::cout << src_to_dst_htm << "\n";
}
