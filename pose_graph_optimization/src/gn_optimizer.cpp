#include <iostream>
#include <ctime>
#include <vector>
#include <memory>
#include <algorithm>
#include <Eigen/Dense>

#include "gn_optimizer.hpp"
#include "gn_helper.hpp"
#include "utils.hpp"

namespace GN
{
    // Constructor
    GnOptimizer::GnOptimizer(
        const std::vector<std::unique_ptr<GN::Mat33>> &Z,
        const std::vector<std::unique_ptr<utils::Node>> &N,
        const GN::GN_Config config)
        : config_(config)
    {
        /*
         * UNIQUE_PTR CHANGE:
         *  - We can’t copy unique_ptr directly (copying is forbidden).
         *  - So we manually deep-copy the pointed-to objects into Z_ and N_.
         */

        // Deep copy each Mat33
        Z_.reserve(Z.size());
        for (const auto &ptr : Z)
        {
            // Copy the underlying matrix into our owned vector
            Z_.push_back(*ptr);   // dereference unique_ptr → Mat33 copy
        }

        // Deep copy each Node
        N_.reserve(N.size());
        for (const auto &ptr : N)
        {
            N_.push_back(*ptr);   // dereference → copy Node
        }

        // fix pointer reassignment
        gn_helper::remapParentPointers(N, N_);

        // Initalize copy of nodes backwards
        X_ = N_;
        std::reverse(X_.begin(), X_.end());
    }

    std::pair<GN::Vec3, GN::Mat36> 
    GnOptimizer::computeErrorAndJacobian(const GN::Vec3 &xi, 
                                    const GN::Vec3 &xj, 
                                    const GN::Mat33 &z_ji)
    {
        /*
        * Compute the error and Jacobian for adjacent nodes i,j
        *
        * @param xi - ith pose (previous)
        * @param xj - jth pose (current)
        * @param zji - icp transform btwn ij (3x3)
        * 
        * @return e, J - error vec and Jacobian mat
        */

        // initalize error and jacobian return struct for current adjancent nodes
        GN::Vec3 e;

        // decpmse poses
        double xi_x = xi(0), xi_y = xi(1), xi_th = xi(2);
        double xj_x = xj(0), xj_y = xj(1), xj_th = xj(2);

        // Rotation of node i. NOTE this is R(theta_i) TRANSPOSED -- it rotates a
        // world-frame vector into node i's local frame.
        Eigen::Matrix2d Ri;
        Ri << cos(xi_th),  sin(xi_th),
            -sin(xi_th),  cos(xi_th);

        // ICP gives the relative pose of j expressed in i's frame. Decompose it
        // into a translation and a rotation so the residual can be formed in the
        // same local frame the Jacobian below is derived in.
        GN::Vec3 z = gn_helper::matToPose(z_ji);
        Eigen::Vector2d t_z(z(0), z(1));

        // Displacement between the two poses, in the world frame
        Eigen::Vector2d dt(xj_x - xi_x, xj_y - xi_y);

        // Build the vector error, in node i's local frame:
        //   e_trans = R(theta_i)^T (t_j - t_i) - t_z
        //   e_theta = (theta_j - theta_i) - theta_z
        // This is the formulation the Jacobian below differentiates. Computing a
        // global-frame prediction error here instead makes J the derivative of a
        // *different* function, which sends Gauss-Newton the wrong way.
        Eigen::Vector2d e_trans = Ri * dt - t_z;

        e << e_trans(0),
            e_trans(1),
            utils::wrap_rad(xj_th - xi_th - z(2));

        // compute jacobian
        GN::Mat36 J;
        J.setZero();

        // d e_trans / d xi
        J.block<2,2>(0,0) = -Ri;

        // d e_trans / d xj
        J.block<2,2>(0,3) = Ri;

        // d e_trans / d theta_i = d(R(theta_i)^T)/d theta_i * (t_j - t_i)
        Eigen::Matrix2d dRiT_dtheta;
        dRiT_dtheta << -sin(xi_th),  cos(xi_th),
                       -cos(xi_th), -sin(xi_th);

        Eigen::Vector2d dtrans_dtheta_i = dRiT_dtheta * dt;
        J(0,2) = dtrans_dtheta_i(0);
        J(1,2) = dtrans_dtheta_i(1);

        // d e_theta / d xi
        J(2,2) = -1;

        // d e_theta / d xj
        J(2,5) = 1;

        return {e, J};
    }

    double GnOptimizer::buildLinearHb(const size_t n,
                        // const std::vector<utils::Node> &X,
                        GN::dMat &H,
                        GN::dVec &b)
    {
        /*
        * Build the coeffient H matrix and b vector for all nodes
        *
        * @param n - length
        * @param X - vector of poses copied from nodes (N) vector
        * @param H - empty size-defined sparse matrix to build
        * @param b - empty size-defined vector to build
        *
        * @return chi2 - total squared error over every edge
        */

        double chi2 = 0.0;

        for (size_t k = 0; k < n; k++)
        {
            int i = k+1;
            int j = k;

            // Set poses as vectors
            GN::Vec3 xi(X_[i].pose.x, X_[i].pose.y, X_[i].pose.theta);
            GN::Vec3 xj(X_[j].pose.x, X_[j].pose.y, X_[j].pose.theta);
            
            // Get the error vec and Jacobian mat
            auto [e, J] = computeErrorAndJacobian(xi, xj, Z_[k]);

            // Accumulate this edge's contribution to the global squared error.
            // A single low-error edge means nothing on its own -- the robot is
            // stationary at the start and end of the run, so those edges are
            // legitimately ~0 while the middle of the graph is badly drifted.
            chi2 += e.transpose() * config_.omega * e;

            Eigen::Matrix<double,6,6> H_ij = J.transpose() * config_.omega * J;
            Eigen::Matrix<double,6,1> b_ij = -J.transpose() * config_.omega * e;

            // Fill into global H and b
            H.block<3,3>(3*i, 3*i) += H_ij.block<3,3>(0,0);
            H.block<3,3>(3*i, 3*j) += H_ij.block<3,3>(0,3);
            H.block<3,3>(3*j, 3*i) += H_ij.block<3,3>(3,0);
            H.block<3,3>(3*j, 3*j) += H_ij.block<3,3>(3,3);

            b.segment<3>(3*i) += b_ij.segment<3>(0);
            b.segment<3>(3*j) += b_ij.segment<3>(3);
            }

            // Gauge freedom: relative constraints alone pin the graph's *shape*
            // but not where it sits, so H is rank-deficient by 3 (global x, y,
            // theta) and LDLT would return garbage. Anchor one pose with a
            // strong prior to make the solution unique. X_ is stored reversed,
            // so the last element is the chronological start of the run -- we
            // pin that and let the accumulated drift be corrected downstream.
            const size_t anchor = X_.size() - 1;
            H.block<3,3>(3*anchor, 3*anchor) += Eigen::Matrix3d::Identity() * 1e6;

            return chi2;
    }

    bool GnOptimizer::gnOptimizer()
    {
        /*
        * Run the gauss newton global optimizer.
        *
        * @param Z - vector of 3x3 homogeneous matrix from ICP
        * @param N - vector of nodes, will extract 3x3 homogeneous matrix from eddge odom
        * @param config
        * 
        * @return bool - true/false for if optimizer has finished (threshold?)
        */

        // Runtime debugger
        clock_t start = std::clock();

        std::cout << "Running Gauss-Newton Optimization..." << std::endl;

        size_t n = Z_.size();

        // The system has one 3-DOF block per POSE, not per edge. The edge loop
        // touches indices i = k+1 up to n, so sizing by Z_.size() leaves the
        // last pose's block off the end of H and b.
        const size_t dim = 3 * X_.size();

        // Allocate space to copy nodes in reverse order
        // We don't want to edit the nodes diretly
        // std::vector<utils::Node> X = N_;
        // std::reverse(X.begin(), X.end());

        for (size_t j = 0; j < config_.max_iters; j++)
        {
            // Intialize empty matrix and vector
            GN::dMat H = GN::dMat::Zero(dim, dim);
            GN::dVec b = GN::dVec::Zero(dim);

            // Extract the H adjancency matrix and b coefficient vector
            double chi2 = buildLinearHb(n, H, b);

            std::cout << "  iter " << j << "  chi2 = " << chi2 << std::endl;

            // Checking for convergence on the WHOLE graph
            if (chi2 < config_.threshold)
            {
                std::cout << "OPTIMIZATION MET THRESHOLD" << std::endl;
                break;
            }

            // Solving for state delta X (x,y,theta)
            Eigen::LDLT<GN::dMat> ldlt(H);
            GN::dVec dX = ldlt.solve(b);

            /////////// Debugging //////////////
            // check linear solver
            if (ldlt.info() != Eigen::Success) {
                std::cerr << "LDLT solve failed :(" << std::endl;
                return false;
            }
            // check size
            if (dX.size() != 3 * X_.size()) {
                std::cerr << "ERROR: dX has wrong size. Expected "
                        << 3 * X_.size() << " but got " << dX.size() << std::endl;
                return false;
            }
            if (!dX.allFinite()) {
                std::cerr << "ERROR: dX contains NaN/Inf -- H is singular" << std::endl;
                return false;
            }
            ////////////////////////////////////

            // Update state vectors
            for (size_t i = 0; i < X_.size(); i++)
            {
                X_[i].pose.x += dX.segment<3>(3*i)(0);
                X_[i].pose.y += dX.segment<3>(3*i)(1);
                X_[i].pose.theta = utils::wrap_rad(X_[i].pose.theta + dX.segment<3>(3*i)(2));
            }

            // Converged when the step itself stops moving the graph
            if (dX.norm() < 1e-6) {
                std::cout << "OPTIMIZATION CONVERGED (step size)" << std::endl;
                break;
            }
        }
        
        // Finished
        double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        std::cout << "Gauss-Newton Optimization Complete!" << std::endl;
        std::cout << "Optimization took " << duration << "secs.\n" << std::endl;
        return true;

        // return false somewhere

    }    
}



