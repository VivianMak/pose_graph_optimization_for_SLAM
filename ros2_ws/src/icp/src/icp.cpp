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

    for (size_t i = 0; i < 1 && i < laser_scans->size(); ++i) {
        std::cout << "------- scan " << i << " ------------\n";
        for (LaserScan& scan : laser_scans->at(i)) {
            std::cout << "angle: " << scan.angle_rad
            << " distance: " << scan.distance
            << " x: " << scan.x
            << " y: " << scan.y << "\n";
        }
        std::cout << "\n";
    }

    std::cout << laser_scans->at(0).size() << "\n";
}
