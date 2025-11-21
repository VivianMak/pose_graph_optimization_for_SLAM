#include <string>
#include <memory>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

struct LaserScan
{
    double angle_rad, distance, x, y;
};

struct Pose {
    double x;
    double y;
    double theta;       // heading of robot
};

void read_odom(std::string file_path, std::vector<Pose> &pose_vector);
void read_lidar(std::string file_path, std::vector<std::vector<LaserScan>> &lidar_ptr);
Eigen::MatrixXd scan_to_matrix(std::vector<LaserScan> single_scan);
Eigen::Matrix3d pose_to_htm(Pose pose);