#include <iostream>
#include "read_data.hpp"
#include "test_structs.hpp"

int main() {
    std::cout << "Reading binary robot data...\n";

    auto [scans, odoms] = loadData("../../data/robot_data.bin");

    // std::cout << "Loaded " << scans.size() << " lidar scans\n";
    // std::cout << "Loaded " << odoms.size() << " odometry entries\n";

    std::cout << "\nRunning struct tests...\n";
    testStructs();

    return 0;
}
