#include <string>
#include <memory>
#include <vector>
#include <iostream>

struct LaserScan
{
    double angle_rad, distance, x, y;
};

void read_odom(std::string file_path, std::vector<double> &x_ptr, std::vector<double> &y_ptr, std::vector<double> &theta_ptr);
void read_lidar(std::string file_path, std::vector<std::vector<LaserScan>> &lidar_ptr);