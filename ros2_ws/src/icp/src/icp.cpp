#include "icp.h"

int main() {
    int scan_one_idx = 0;
    int scan_two_idx = 2695;

    auto noisy_poses = std::make_unique<std::vector<Pose>>();

    read_odom("noisy_odom_data.csv", *noisy_poses);

    auto laser_scans = std::make_unique<std::vector<std::vector<LaserScan>>>();
    
    read_lidar("lidar_scans.csv", *laser_scans);

    Eigen::MatrixXd src_point_matrix = scan_to_matrix(laser_scans->at(scan_two_idx));
    Eigen::MatrixXd dst_point_matrix = scan_to_matrix(laser_scans->at(scan_one_idx));

    Eigen::MatrixXd odom_htm = htm_between_poses(
        noisy_poses->at(scan_one_idx),
        noisy_poses->at(scan_two_idx)
    );

    Eigen::MatrixXd normals = compute_normals(dst_point_matrix, 10);
    std::cout << normals.rows() << " " << normals.cols() << "\n";
}
