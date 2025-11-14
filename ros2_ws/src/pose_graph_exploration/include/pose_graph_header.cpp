#include "pose_graph_header.h"
#include <cmath>

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

std::vector<double> quaternion2euler(double x, double y, double z, double w) {
    std::vector<double> angles;
    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    angles.push_back(std::atan2(sinr_cosp, cosr_cosp));

    // Pitch (y-axis rotation) - handle potential gimbal lock
    double sinp = 2 * (w * y - z * x);
    if (std::abs(sinp) >= 1) {
        angles.push_back(std::copysign(M_PI / 2, sinp)); // Use 90 degrees if close to gimbal lock
    } else {
        angles.push_back(std::asin(sinp));
    }

    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    angles.push_back(std::atan2(siny_cosp, cosy_cosp));

    return angles;
}

Eigen::Matrix4d pose2homogeneous(double x, double y, double theta)
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();

    double c = std::cos(theta);
    double s = std::sin(theta);

    T(0,0) = c;   T(0,1) = -s;  T(0,3) = x;
    T(1,0) = s;   T(1,1) =  c;  T(1,3) = y;

    return T;
}