#include "pose_graph_header.h"

void read_data(std::string filename, std::shared_ptr<std::vector<SavedLaserScan>> lidar_vector, std::shared_ptr<std::vector<SavedOdom>> odom_vector) {
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Error: Could not open file robot_data.bin\n";
        return;
    }

    // --- Load Laser Scans ---
    uint64_t scanCount;
    file.read(reinterpret_cast<char*>(&scanCount), sizeof(scanCount));
    (*lidar_vector).resize(scanCount);

    for (uint64_t i = 0; i < scanCount; i++) {
        uint64_t n;
        file.read(reinterpret_cast<char*>(&n), sizeof(n));
        (*lidar_vector)[i].ranges.resize(n);
        file.read(reinterpret_cast<char*>((*lidar_vector)[i].ranges.data()), n * sizeof(float));
        file.read(reinterpret_cast<char*>(&(*lidar_vector)[i].sec), sizeof(int32_t));
        file.read(reinterpret_cast<char*>(&(*lidar_vector)[i].nanosec), sizeof(uint32_t));
    }

    // --- Load Odom Data ---
    uint64_t odomCount;
    file.read(reinterpret_cast<char*>(&odomCount), sizeof(odomCount));

    (*odom_vector).resize(odomCount);

    for (uint64_t i = 0; i < odomCount; i++) {
        // Read header values
        file.read(reinterpret_cast<char*>(&(*odom_vector)[i].sec), sizeof(int32_t));
        file.read(reinterpret_cast<char*>(&(*odom_vector)[i].nanosec), sizeof(uint32_t));

        // Read pose values
        file.read(reinterpret_cast<char*>(&(*odom_vector)[i].position_x), sizeof(double));
        file.read(reinterpret_cast<char*>(&(*odom_vector)[i].position_y), sizeof(double));
        file.read(reinterpret_cast<char*>(&(*odom_vector)[i].position_z), sizeof(double));
        file.read(reinterpret_cast<char*>(&(*odom_vector)[i].orientation_x), sizeof(double));
        file.read(reinterpret_cast<char*>(&(*odom_vector)[i].orientation_y), sizeof(double));
        file.read(reinterpret_cast<char*>(&(*odom_vector)[i].orientation_z), sizeof(double));
        file.read(reinterpret_cast<char*>(&(*odom_vector)[i].orientation_w), sizeof(double));

        // Read twist values
        file.read(reinterpret_cast<char*>(&(*odom_vector)[i].linear_x), sizeof(double));
        file.read(reinterpret_cast<char*>(&(*odom_vector)[i].linear_y), sizeof(double));
        file.read(reinterpret_cast<char*>(&(*odom_vector)[i].linear_z), sizeof(double));
        file.read(reinterpret_cast<char*>(&(*odom_vector)[i].angular_x), sizeof(double));
        file.read(reinterpret_cast<char*>(&(*odom_vector)[i].angular_y), sizeof(double));
        file.read(reinterpret_cast<char*>(&(*odom_vector)[i].angular_z), sizeof(double));
    }

    file.close();

    std::cout << "Loaded " << (*lidar_vector).size() << " lidar scans\n";
    std::cout << "Loaded " << (*odom_vector).size() << " odom points\n";
}