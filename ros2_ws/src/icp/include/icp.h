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
Eigen::MatrixXd htm_between_poses(Pose pose_1, Pose pose_2);
Eigen::Vector2d get_smallest_ev(const Eigen::MatrixXd& pts);
Eigen::MatrixXd compute_normals(Eigen::MatrixXd dst_points, size_t num_neighbors);
std::vector<std::ptrdiff_t> make_correspondences(Eigen::MatrixXd src_points, Eigen::MatrixXd dst_points);
Eigen::Matrix3d least_squares_transform(Eigen::MatrixXd src_points, Eigen::MatrixXd dst_points, Eigen::MatrixXd normals, double error_weight);