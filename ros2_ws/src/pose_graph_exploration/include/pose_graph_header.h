#include <vector>
#include <cstdint>
#include <string>
#include <fstream>
#include <iostream>
#include <memory>
#include <Eigen/Dense>

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

class Node {
public:
    // Pose data
    float x;
    float y;
    float theta;

    // List of laser scan points (Cartesian coordinates)
    std::vector<double> distances;

    // Transformation matrix (3x3)
    Eigen::Matrix4d transform_mat;

    // Parent node (previous in sequence)
    Node* parent;

    // Begin constructor
    Node(float x_, float y_, float theta_, Node* parent_ = nullptr)
    : x(x_), y(y_), theta(theta_), parent(parent_) {
    // Initialize transform as identity matrix
    // transform_mat = {{{1.0, 0.0, 0.0},{0.0, 1.0, 0.0},{0.0, 0.0, 1.0}}};
    }
};

void read_data(std::string filename, std::shared_ptr<std::vector<SavedLaserScan>> lidar_vector, std::shared_ptr<std::vector<SavedOdom>> odom_vector);
std::vector<double> quaternion2euler(double x, double y, double z, double w);
Eigen::Matrix4d pose2homogeneous(double x, double y, double z);