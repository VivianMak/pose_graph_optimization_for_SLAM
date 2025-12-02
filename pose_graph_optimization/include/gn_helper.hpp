#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace gn_helper {

Eigen::Vector3d matToPose(const Eigen::Matrix3d& T)
{
    Eigen::Vector3d pose;
    pose.x() = T(0, 2);                        
    pose.y() = T(1, 2);                       
    pose.z() = std::atan2(T(1, 0), T(0, 0));
    return pose;
}

Eigen::Matrix3d poseToMat(const Eigen::Vector3d& p)
{
    double c = std::cos(p.z());
    double s = std::sin(p.z());

    Eigen::Matrix3d T;
    T <<  c, -s, p.x(),
          s,  c, p.y(),
          0,  0, 1;
    return T;
}

}
