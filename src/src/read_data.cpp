#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>

struct SavedLaserScan 
{
    std::vector<float> ranges;  // ranges
    int32_t sec;   // header.stamp.sec
    uint32_t nanosec;   // header.stamp.nanosec
};

struct SavedOdom
{
    int32_t sec;   // header.stamp.sec
    uint32_t nanosec;   // header.stamp.nanosec

    double position_x;  // pose.pose.position.x
    double position_y;  // pose.pose.position.y
    double position_z;  // pose.pose.position.z

    double orientation_x; // pose.pose.orientation.x
    double orientation_y; // pose.pose.orientation.y
    double orientation_z; // pose.pose.orientation.z
    double orientation_w; // pose.pose.orientation.w

    double linear_x;  // twist.twist.linear.x
    double linear_y;  // twist.twist.linear.y
    double linear_z;  // twist.twist.linear.z

    double angular_x;  // twist.twist.angular.x
    double angular_y;  // twist.twist.angular.y
    double angular_z;  // twist.twist.angular.z
};

int main() {
    std::ifstream file("../../data/robot_data.bin", std::ios::binary);
    if (!file) {
        std::cerr << "Error: Could not open file robot_data.bin\n";
        return 1;
    }

    std::vector<SavedLaserScan> scans;
    std::vector<SavedOdom> odoms;

    // --- Load Laser Scans ---
    uint64_t scanCount;
    file.read(reinterpret_cast<char*>(&scanCount), sizeof(scanCount));
    scans.resize(scanCount);

    for (uint64_t i = 0; i < scanCount; i++) {
        uint64_t n;
        file.read(reinterpret_cast<char*>(&n), sizeof(n));
        scans[i].ranges.resize(n);
        file.read(reinterpret_cast<char*>(scans[i].ranges.data()), n * sizeof(float));
        file.read(reinterpret_cast<char*>(&scans[i].sec), sizeof(int32_t));
        file.read(reinterpret_cast<char*>(&scans[i].nanosec), sizeof(uint32_t));
    }

    // --- Load Odom Data ---
    uint64_t odomCount;
    file.read(reinterpret_cast<char*>(&odomCount), sizeof(odomCount));

    odoms.resize(odomCount);

    for (uint64_t i = 0; i < odomCount; i++) {
        // Read header values
        file.read(reinterpret_cast<char*>(&odoms[i].sec), sizeof(int32_t));
        file.read(reinterpret_cast<char*>(&odoms[i].nanosec), sizeof(uint32_t));

        // Read pose values
        file.read(reinterpret_cast<char*>(&odoms[i].position_x), sizeof(double));
        file.read(reinterpret_cast<char*>(&odoms[i].position_y), sizeof(double));
        file.read(reinterpret_cast<char*>(&odoms[i].position_z), sizeof(double));
        file.read(reinterpret_cast<char*>(&odoms[i].orientation_x), sizeof(double));
        file.read(reinterpret_cast<char*>(&odoms[i].orientation_y), sizeof(double));
        file.read(reinterpret_cast<char*>(&odoms[i].orientation_z), sizeof(double));
        file.read(reinterpret_cast<char*>(&odoms[i].orientation_w), sizeof(double));

        // Read twist values
        file.read(reinterpret_cast<char*>(&odoms[i].linear_x), sizeof(double));
        file.read(reinterpret_cast<char*>(&odoms[i].linear_y), sizeof(double));
        file.read(reinterpret_cast<char*>(&odoms[i].linear_z), sizeof(double));
        file.read(reinterpret_cast<char*>(&odoms[i].angular_x), sizeof(double));
        file.read(reinterpret_cast<char*>(&odoms[i].angular_y), sizeof(double));
        file.read(reinterpret_cast<char*>(&odoms[i].angular_z), sizeof(double));
    }

    file.close();

    std::cout << "Loaded " << scans.size() << " lidar scans\n";
    std::cout << "Loaded " << odoms.size() << " odom points\n";

    return 0;
}