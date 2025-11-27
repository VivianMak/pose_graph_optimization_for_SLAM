#include "pose_graph.hpp"
#include <cmath>
#include <iostream>

namespace pose_graph {

PoseGraph::PoseGraph()
{
    // Constructor with empty function, thresholds can be added later if needed.
}

void PoseGraph::build(const std::vector<SavedLaserScan>& scans,
                      const std::vector<SavedOdom>& odoms)
{
    /*
     * Build the pose graph from odometry and scans.
     *
     * Currently only odometry is used to construct poses.
     * Scans are reserved for future loop-closure or matching steps.
     *
     * @param scans  lidar scan data (unused for now)
     * @param odoms  wheel odometry data used to create Nodes
     * @return void
     */

    (void)scans;  // unused for now

    nodes_.clear();
    edge_indices_.clear();

    // Create one Node per odometry entry
    for (std::size_t i = 0; i < odoms.size(); ++i) {
        utils::Pose p = odomToPose(odoms[i]);
        nodes_.push_back(std::make_unique<utils::Node>(static_cast<int>(i), p));
    }

    // Connect consecutive nodes with edges
    addSequentialEdges();
}

const std::vector<std::unique_ptr<utils::Node>>& PoseGraph::nodes() const
{
    /*
     * Return reference to internal node vector.
     *
     * @return vector of unique_ptr<Node>
     */
    return nodes_;
}

const std::vector<EdgeIndex>& PoseGraph::edgeIndices() const
{
    /*
     * Return simple parent-child index pairs for edges.
     *
     * @return vector of EdgeIndex structs
     */
    return edge_indices_;
}

utils::Pose PoseGraph::odomToPose(const SavedOdom& odom) const
{
    /*
     * Convert odometry quaternion to a 2D pose.
     *
     * Extracts x, y directly. Computes yaw from quaternion, converts to degrees.
     *
     * @param odom  odometry entry
     * @return Pose (x, y, theta_deg)
     */

    double x = odom.position_x;
    double y = odom.position_y;

    // quaternion → yaw
    double qx = odom.orientation_x;
    double qy = odom.orientation_y;
    double qz = odom.orientation_z;
    double qw = odom.orientation_w;

    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    double yaw_rad = std::atan2(siny_cosp, cosy_cosp);
    double yaw_deg = yaw_rad * 180.0 / M_PI;

    return utils::Pose(x, y, yaw_deg);
}

void PoseGraph::addSequentialEdges()
{
    /*
     * Add edges between nodes i to i+1.
     *
     * Computes relative transform via delta pose and stores edges
     * inside the parent Node. Also fills the edge index list for
     * external inspection.
     *
     * @return void
     */

    if (nodes_.size() < 2) {
        return;
    }

    for (std::size_t i = 0; i + 1 < nodes_.size(); ++i) {
        utils::Node* parent = nodes_[i].get();
        utils::Node* child  = nodes_[i + 1].get();

        utils::Pose delta = child->pose - parent->pose;

        // convert heading delta into radians for trig
        double theta_rad = delta.theta * M_PI / 180.0;

        Eigen::Matrix3d T;
        T << std::cos(theta_rad), -std::sin(theta_rad), delta.x,
             std::sin(theta_rad),  std::cos(theta_rad), delta.y,
             0.0,                  0.0,                 1.0;

        parent->addEdge(child, T);

        EdgeIndex idx{ i, i + 1 };
        edge_indices_.push_back(idx);
    }
}

} 
