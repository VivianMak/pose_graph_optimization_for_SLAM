#include "icp.h"

int main() {
    auto noisy_xs = std::make_unique<std::vector<double>>();
    auto noisy_ys = std::make_unique<std::vector<double>>();
    auto noisy_thetas = std::make_unique<std::vector<double>>();

    read_odom("noisy_odom_data.csv", *noisy_xs, *noisy_ys, *noisy_thetas);

    for (size_t i = 0; i < 5 && i < (*noisy_xs).size(); ++i)
        std::cout << (*noisy_xs)[i] << ", " << (*noisy_ys)[i] << ", " << (*noisy_thetas)[i] << "\n";

    auto laser_scans = std::make_unique<std::vector<std::vector<LaserScan>>>();
    
    read_lidar("lidar_scans.csv", *laser_scans);

    Eigen::MatrixXd laser_scan_matrix = scan_to_matrix(laser_scans->at(0));
    std::cout << laser_scan_matrix.rows() << " " << laser_scan_matrix.cols() << "\n";
}
