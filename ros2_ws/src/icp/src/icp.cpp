#include "icp.h"

int main() {
    int scan_one_idx = 0;
    int scan_two_idx = 2695;

    auto noisy_poses = std::make_unique<std::vector<Pose>>();

    read_odom("noisy_odom_data.csv", *noisy_poses);

    for (size_t i = 0; i < 5 && i < noisy_poses->size(); ++i)
        std::cout << noisy_poses->at(i).x << ", " << noisy_poses->at(i).y << ", " << noisy_poses->at(i).theta << "\n";

    auto laser_scans = std::make_unique<std::vector<std::vector<LaserScan>>>();
    
    read_lidar("lidar_scans.csv", *laser_scans);

    Eigen::MatrixXd laser_scan_matrix = scan_to_matrix(laser_scans->at(0));
    std::cout << laser_scan_matrix.rows() << " " << laser_scan_matrix.cols() << "\n";

    Eigen::MatrixXd one_to_two_htm = htm_between_poses(
        noisy_poses->at(scan_one_idx),
        noisy_poses->at(scan_two_idx)
    );

    std::cout << one_to_two_htm << "\n";
}
