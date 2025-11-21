#include "icp.h"
#include <fstream>
#include <sstream>
#include <cmath>

void read_odom(std::string file_path, std::vector<Pose> &pose_vector) {
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
        pose_vector.push_back({v[0], v[1], v[2]});
    }

    return;
}

void read_lidar(std::string file_path, std::vector<std::vector<LaserScan>> &lidar_ptr) {
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
        std::vector<LaserScan> single_laser_scan;
        int i = 0;
        int num_scans = 640;
        while (std::getline(ss, val, ',') && i < num_scans) {
            double angle = i * 2 * M_PI / (num_scans - 1); // in radians
            double distance = std::stod(val);
            if (std::isinf(distance)) distance = 0.0;
            single_laser_scan.push_back({angle, distance, distance * std::cos(angle), distance * std::sin(angle)});
            i++;
        }
        lidar_ptr.push_back(single_laser_scan);
    }
}   

Eigen::MatrixXd scan_to_matrix(std::vector<LaserScan> single_scan) {
    size_t num_points = single_scan.size();
    Eigen::MatrixXd mat(3, num_points);

    for (size_t idx = 0; idx < num_points; ++idx) {
        mat(0, idx) = single_scan[idx].x;
        mat(1, idx) = single_scan[idx].y;
        mat(2, idx) = 1;
    }
    
    return mat;
}

Eigen::Matrix3d pose_to_htm(Pose pose) {
    double c = std::cos(pose.theta);
    double s = std::sin(pose.theta);

    Eigen::Matrix3d T;
    T << c, -s, pose.x,
         s,  c, pose.y,
         0,  0, 1;
    return T;
}

