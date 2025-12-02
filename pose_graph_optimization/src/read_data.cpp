#include <iostream>
#include <fstream>
#include <vector>
#include <cstdint>

#include "read_data.hpp"

// Initialize the vector pair 
// allows us to return both variables with synced up time
std::pair<std::vector<SavedLaserScan>, std::vector<SavedOdom>>

loadData(const std::string& FILENAME)
{
    /*
    * Load a binary data file taken from ROS2 sim.
    *
    * @param FILENAME (string)- the .bin data file
    * @return {odoms, scans} (vector tuple)- wheel odometry and laser scan at a timestep
    */

    std::vector<SavedLaserScan> scans;
    std::vector<SavedOdom> odoms;


    std::ifstream file(FILENAME, std::ios::binary);
    if (!file) {
        std::cerr << "Error: Could not open file robot_data.bin\n";
        return {scans, odoms}; // empty
    }
   

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

    // Verify the vectors
    std::cout << "Loaded " << scans.size() << " lidar scans\n";
    std::cout << "Loaded " << odoms.size() << " odom points\n";

    return {scans, odoms};
}