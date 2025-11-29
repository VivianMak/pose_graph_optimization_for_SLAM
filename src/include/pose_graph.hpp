#pragma once

#include <vector>
#include <memory>
#include <cstddef>    
#include <Eigen/Dense>

#include "utils.hpp"
#include "read_data.hpp"

namespace pose_graph {

// -------------------------
// EdgeIndex: simple parent-child index pair
// -------------------------
struct EdgeIndex {
    std::size_t parent_index;
    std::size_t child_index;
};

// -------------------------
// PoseGraph
// -------------------------
//
// Builds a basic pose graph from odometry + lidar input using utils::Node.
// Produces node storage and a simple list of which node indices connect.
//
class PoseGraph {
public:
    /**
     * Builds and initializes an empty pose graph.
     *
     * No parameters; prepares containers for nodes and edges.
     */
    PoseGraph();

    /**
     * Build the pose graph from lidar scans and odometry data.
     *
     * Creates one Node per odometry entry. Each node stores a 2D pose.
     * Adds sequential edges between consecutive nodes using relative transforms.
     *
     * @param scans  (vector) lidar scans loaded from read_data
     * @param odoms  (vector) odometry entries loaded from read_data
     * @return void
     */
    void build(const std::vector<SavedLaserScan>& scans,
               const std::vector<SavedOdom>& odoms);

    /**
     * Get reference to internal node list.
     *
     * Nodes are stored as unique_ptr<utils::Node>.
     *
     * @return vector of owned Node pointers
     */
    const std::vector<std::unique_ptr<utils::Node>>& nodes() const;

    /**
     * Get index pairs representing edges in the graph.
     *
     * Each EdgeIndex {parent_index, child_index} tells which nodes are connected.
     *
     * @return vector of edge index pairs
     */
    const std::vector<EdgeIndex>& edgeIndices() const;

private:
    /**
     * Convert a SavedOdom structure to a 2D Pose.
     *
     * Extracts x, y from position and yaw from quaternion orientation.
     * Yaw is converted to degrees because utils::Pose stores theta in degrees.
     *
     * @param odom  (SavedOdom) one odometry reading
     * @return Pose with x, y, theta_deg
     */
    utils::Pose odomToPose(const SavedOdom& odom) const;

    /**
     * Add sequential edges between consecutive nodes.
     *
     * Computes relative transforms and stores edges inside each parent Node.
     * Also records an index list for external iteration.
     *
     * @return void
     */
    void addSequentialEdges();

    // Graph storage
    std::vector<std::unique_ptr<utils::Node>> nodes_;   // owned Nodes
    std::vector<EdgeIndex> edge_indices_;               // index connections
};

} // namespace pose_graph
