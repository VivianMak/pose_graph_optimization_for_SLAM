#include "icp.h"
#include <fstream>
#include <sstream>

void read_odom(std::string file_path, std::unique_ptr<std::vector<double>> &x_ptr, std::unique_ptr<std::vector<double>> &y_ptr, std::unique_ptr<std::vector<double>> &theta_ptr) {
    std::ifstream file(file_path);
    if (!file) {
        std::cerr << "Cannot open file.\n";
        return;
    }

    std::string line;

    // Skip header
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string val;
        double v[3];
        int i = 0;
        while (std::getline(ss, val, ',') && i < 3) {
            v[i++] = std::stod(val);
        }
        x_ptr->push_back(v[0]);
        y_ptr->push_back(v[1]);
        theta_ptr->push_back(v[2]);
    }

    return;
}