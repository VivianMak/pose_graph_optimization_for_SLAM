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

int main() {
    std::ifstream file("lidar_data.bin", std::ios::binary);
    if (!file) {
        std::cerr << "Error: Could not open file lidar_data.bin\n";
        return 1;
    }

    std::vector<SavedLaserScan> scans;

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

    file.close();

    std::cout << "Loaded " << scans.size() << " lidar scans\n";

    if (!scans.empty()) {
        for (size_t scan_idx = 0; scan_idx < scanCount; scan_idx += 100){
            // n is the number of distances in one lidar scan
            // uint64_t n;
            // file.read(reinterpret_cast<char*>(&n), sizeof(n));
            std::cout << "Laser scan " << scan_idx << "\n";
            std::cout << "  ranges = [ ";
            for (size_t i = 0; i < 10; i++) {
                std::cout << scans[scan_idx].ranges[i] << " ";
            }
            std::cout << "]\n";
            std::cout << "  timestamp = " << scans[scan_idx].sec << "." << scans[scan_idx].nanosec << "\n";
        }
        
    }

    return 0;
}