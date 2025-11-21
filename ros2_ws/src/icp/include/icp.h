#include <string>
#include <memory>
#include <vector>
#include <iostream>

void read_odom(std::string file_path, std::unique_ptr<std::vector<double>> &x_ptr, std::unique_ptr<std::vector<double>> &y_ptr, std::unique_ptr<std::vector<double>> &theta_ptr);
void read_lidar(std::string file_path, std::unique_ptr<std::vector<std::vector<double>>> &lidar_ptr);