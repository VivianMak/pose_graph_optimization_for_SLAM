#include "icp.h"

int main() {
    int scan_one_idx = 0;
    // int scan_two_idx = 2695;

    auto noisy_poses = std::make_unique<std::vector<Pose>>();

    read_odom("noisy_odom_data.csv", *noisy_poses);

    for (size_t i = 0; i < 5 && i < noisy_poses->size(); ++i)
        std::cout << noisy_poses->at(i).x << ", " << noisy_poses->at(i).y << ", " << noisy_poses->at(i).theta << "\n";

    auto laser_scans = std::make_unique<std::vector<std::vector<LaserScan>>>();
    
    read_lidar("lidar_scans.csv", *laser_scans);

    Eigen::MatrixXd laser_scan_matrix = scan_to_matrix(laser_scans->at(0));
    std::cout << laser_scan_matrix.rows() << " " << laser_scan_matrix.cols() << "\n";

    Eigen::MatrixXd odom_one_htm = pose_to_htm(
        noisy_poses->at(scan_one_idx)
    );

    std::cout << odom_one_htm << "\n";
}
