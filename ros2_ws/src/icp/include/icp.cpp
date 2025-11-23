#include "icp.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <nanoflann.hpp>

void read_odom(std::string file_path, std::vector<Pose> &pose_vector) {
    std::ifstream file(file_path);
    if (!file) {
        std::cerr << "Cannot open file.\n";
        return;
    }

    std::string line;

    // Skip header
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string val;
        double v[3];
        int i = 0;
        while (std::getline(ss, val, ',') && i < 3) {
            v[i++] = std::stod(val);
        }
        pose_vector.push_back({v[0], v[1], v[2]});
    }

    return;
}

void read_lidar(std::string file_path, std::vector<std::vector<LaserScan>> &lidar_ptr) {
    std::ifstream file(file_path);
    if (!file) {
        std::cerr << "Cannot open file.\n";
        return;
    }

    std::string line;

    // Skip header
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string val;
        std::vector<LaserScan> single_laser_scan;
        int i = 0;
        int num_scans = 640;
        while (std::getline(ss, val, ',') && i < num_scans) {
            double angle = i * 2 * M_PI / (num_scans - 1); // in radians
            double distance = std::stod(val);
            if (std::isinf(distance)) distance = 0.0;
            single_laser_scan.push_back({angle, distance, distance * std::cos(angle), distance * std::sin(angle)});
            i++;
        }
        lidar_ptr.push_back(single_laser_scan);
    }
}   

Eigen::MatrixXd scan_to_matrix(std::vector<LaserScan> single_scan) {
    size_t num_points = single_scan.size();
    Eigen::MatrixXd mat(3, num_points);

    for (size_t idx = 0; idx < num_points; ++idx) {
        mat(0, idx) = single_scan[idx].x;
        mat(1, idx) = single_scan[idx].y;
        mat(2, idx) = 1;
    }
    
    return mat;
}

Eigen::Matrix3d pose_to_htm(Pose pose) {
    double c = std::cos(pose.theta);
    double s = std::sin(pose.theta);

    Eigen::Matrix3d T;
    T << c, -s, pose.x,
         s,  c, pose.y,
         0,  0, 1;
    return T;
}

Eigen::MatrixXd htm_between_poses(Pose pose_1, Pose pose_2) {
    // Computes transform from pose 1 to pose 2
    Eigen::Matrix3d htm1 = pose_to_htm(pose_1);
    Eigen::Matrix3d htm2 = pose_to_htm(pose_2);

    return htm1.inverse() * htm2;
}

Eigen::Vector2d get_smallest_ev(const Eigen::MatrixXd& pts) {
    // pts is m × 2

    // 1. Compute mean
    Eigen::RowVector2d mean = pts.colwise().mean();

    // 2. Center the points
    Eigen::MatrixXd centered = pts.rowwise() - mean;

    // 3. Compute covariance matrix (2 × 2)
    Eigen::Matrix2d cov = (centered.transpose() * centered) / (pts.rows() - 1);

    // 4. Eigen decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eig(cov);

    // 5. Smallest eigenvector = normal direction
    Eigen::Vector2d eigvec = eig.eigenvectors().col(0);  // col(0) = smallest eigenvalue

    return eigvec;
}

Eigen::MatrixXd compute_normals(Eigen::MatrixXd dst_points, size_t num_neighbors) {
    using KDTree = nanoflann::KDTreeEigenMatrixAdaptor<
        Eigen::MatrixXd,
        2              // Dimension, two columns, x and y, for kNN
    >;

    size_t num_points = dst_points.cols();
    Eigen::MatrixXd dst_2d = dst_points.topRows(2).transpose(); // Size (n, 2), KDtree needs to be made from eigen matrix where each row is a point

    KDTree dst_kdtree(2, std::cref(dst_2d), 10 /* leaf size */); // Make the KDtree

    std::vector<std::ptrdiff_t> idx(num_neighbors); // Stores the indices of the nearest neighbors
    std::vector<double> dists(num_neighbors); // Doesn't get used

    Eigen::MatrixXd normals(2, num_points); // Matrix containing the vectors of the normal direction of the sets of nearest neighbors for all points, size (2, n), each column is a normal vector
    Eigen::MatrixXd neighbors(num_neighbors, 2); // The coords of the nearest neighbors
    Eigen::RowVector2d centroid = dst_2d.colwise().mean();

    for (size_t point_idx = 0; point_idx < num_points; point_idx++) { // Compute normal for every point and their NNs
        double query_pt[2] = { dst_2d(point_idx, 0), dst_2d(point_idx, 1) }; // Point we're finding the nearest neighbors of

        dst_kdtree.query(query_pt, num_neighbors, idx.data(), dists.data()); // Find the indices of the nearest neighbors

        for (size_t i = 0; i < num_neighbors; i++) { // Use indices to put neighboring points into neighbors matrix
            neighbors.row(i) = dst_2d.row(idx[i]);
        }

        normals.col(point_idx) = get_smallest_ev(neighbors);

        // Correctly orient normals
        Eigen::Vector2d vec_to_centroid = centroid.transpose() - dst_2d.row(point_idx).transpose(); // Vector from point to centroid

        double dot = normals.col(point_idx).dot(vec_to_centroid); // Dot product between current normal and vector toward centroid

        if (dot > 0)
            normals.col(point_idx) = -normals.col(point_idx); // if dot > 0, normal points *toward* centroid, flip it for consistency
    }
    
    return normals;
}

std::vector<std::ptrdiff_t> make_correspondences(Eigen::MatrixXd src_points, Eigen::MatrixXd dst_points) {
    // Return a vector of the corresponding destination point indices, first index in vector is the index of the dst point that is closest to the first src point
    using KDTree = nanoflann::KDTreeEigenMatrixAdaptor<
        Eigen::MatrixXd,
        2              // Dimension, two columns, x and y, for kNN
    >;

    size_t num_points = src_points.cols();
    Eigen::MatrixXd dst_2d = dst_points.topRows(2).transpose(); // Size (n, 2), KDtree needs to be made from eigen matrix where each row is a point

    KDTree dst_kdtree(2, std::cref(dst_2d), 10 /* leaf size */); // Make the KDtree

    std::vector<std::ptrdiff_t> indices(num_points); // Stores the indices of the closest corresponding point

    for (size_t point_idx = 0; point_idx < num_points; point_idx++) {
        double query_pt[2] = { src_points(0, point_idx), src_points(1, point_idx) }; // Point we're finding the index of the closest corresponding point of

        std::ptrdiff_t out_idx;
        double out_dist; // Doesn't get used
        dst_kdtree.query(query_pt, 1, &out_idx, &out_dist); // Find the index of the closest corresponding point

        indices[point_idx] = out_idx;
    }
    
    return indices;
}

