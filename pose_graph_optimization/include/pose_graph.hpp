#pragma once

#include <vector>
#include <memory>
#include <cstddef>    
#include <Eigen/Dense>

#include "utils.hpp"
#include "read_data.hpp"

extern int STEP_SIZE;

namespace pose_graph {

// -------------------------
// EdgeIndex: simple parent-child index pair
// -------------------------
struct EdgeIndex {
    std::size_t parent_index;
    std::size_t child_index;
};

// -------------------------
// NoiseConfig: simulated wheel-odometry drift
// -------------------------
//
// The simulator's odometry is effectively ground truth, so without this there is
// no drift for the optimizer to correct and the "pre-optimized" route is already
// the right answer.
//
// Real odometry drifts because every incremental motion estimate carries a small
// error and those errors COMPOUND -- a heading error early on swings the entire
// trajectory that follows it. Perturbing absolute x/y would not reproduce that;
// the noise has to corrupt each per-step increment, which is then integrated.
// That is what makes the loop fail to close, which is exactly the error signal
// pose graph optimization exists to remove.
//
struct NoiseConfig {
    bool enabled       = false;  // off keeps the original clean-odometry behavior
    double sigma_xy    = 0.0;    // stddev of per-sample translation error (m)
    double sigma_theta = 0.0;    // stddev of per-sample heading error (rad)
    unsigned int seed  = 42;     // fixed so a run is reproducible
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
     * @param noise  (NoiseConfig) optional simulated drift; default is clean odom
     * @return void
     */
    void build(const std::vector<SavedLaserScan>& scans,
               const std::vector<SavedOdom>& odoms,
               const NoiseConfig& noise = {});

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
