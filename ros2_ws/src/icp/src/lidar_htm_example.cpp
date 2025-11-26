#include "icp.h"

int main() {
    // Read and save odom and lidar data
    auto noisy_poses = std::make_unique<std::vector<Pose>>(); // Size 4153
    auto laser_scans = std::make_unique<std::vector<std::vector<LaserScan>>>(); // Size 4136

    read_odom("noisy_odom_data.csv", *noisy_poses);
    read_lidar("lidar_scans.csv", *laser_scans);

    std::vector<Eigen::Matrix3d> lidar_htms; // Vector of lidar transforms

    size_t step_size = 100;
    size_t num_readings = laser_scans->size(); // Need to use laser_scans because noisy_poses is bigger so will incorrectly index laser_scans if we use the larger size

    // Select ICP parameters
    size_t num_iterations = 100;
    size_t num_neighbors = 10;
    double error_weighting = 0.5;

    for (size_t reading_idx = num_readings - 1; reading_idx > step_size; reading_idx -= step_size) {
        // Readings to find lidar transform between
        size_t src_idx = reading_idx;
        size_t dst_idx = reading_idx - step_size;

        // Convert selected scans into matrices that ICP can be done on
        Eigen::MatrixXd src_point_matrix = scan_to_matrix(laser_scans->at(src_idx)); // 3, n
        Eigen::MatrixXd dst_point_matrix = scan_to_matrix(laser_scans->at(dst_idx)); // 3, n

        // Calculate the odom transform between the selected poses
        Eigen::Matrix3d odom_htm = htm_between_poses(
            noisy_poses->at(dst_idx),
            noisy_poses->at(src_idx)
        ); // 3, 3

        // Run ICP
        Eigen::Matrix3d src_to_dst_htm = icp(src_point_matrix, dst_point_matrix, num_iterations, odom_htm, num_neighbors, error_weighting);

        lidar_htms.push_back(src_to_dst_htm);
    }

    // Print the lidar htm
    for (Eigen::Matrix3d htm : lidar_htms) {
        std::cout << htm << "\n";
    }
    
    std::cout << lidar_htms.size() << "\n";
}
