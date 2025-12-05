#pragma once
#include <Eigen/Dense>
#include <cmath>
#include "utils.hpp"

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


void remapParentPointers(
    const std::vector<std::unique_ptr<utils::Node>> &oldNodes,
    std::vector<utils::Node> &newNodes
){
    // Create mapping: old raw pointer -> new raw pointer
    std::unordered_map<const utils::Node*, utils::Node*> pointerMap;

    for (size_t i = 0; i < oldNodes.size(); i++) {
        pointerMap[oldNodes[i].get()] = &newNodes[i];
    }

    // Fix pointers inside each node's edges
    for (size_t i = 0; i < newNodes.size(); i++) {
        utils::Node &n = newNodes[i];

        for (auto &edge : n.edges) {
            const utils::Node* oldParent = edge.parent;
            if (pointerMap.count(oldParent)) {
                edge.parent = pointerMap[oldParent];  // Reassign pointer
            }
        }
    }
}

}