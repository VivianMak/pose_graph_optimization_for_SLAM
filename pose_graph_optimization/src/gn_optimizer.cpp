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

        // Compute error
        std::cout << "---------- Computing Error ----------" << std::endl;

        // Invert ICP j -> i transform
        GN::Mat33 Z_ij = z_ji.inverse();

        // Turn pose i to a matrix
        GN::Mat33 T_i = gn_helper::poseToMat(xi); 

        // Compute predicted j
        GN::Mat33 T_j_pred = T_i * Z_ij;

        // Turn matrix j into a pose
        GN::Vec3 xj_pred = gn_helper::matToPose(T_j_pred);

        // Build the vector error
        e << xj_pred(0) - xj(0),
            xj_pred(1) - xj(1),
            utils::wrap_rad(xj_pred(0) - xj(0));

        std::cout << "ERROR IS: " << e << std::endl;

        // Compute Jacobian
        std::cout << "---------- Computing Jacobian ----------" << std::endl;

        // decpmse poses
        double xi_x = xi(0), xi_y = xi(1), xi_th = xi(2);
        double xj_x = xj(0), xj_y = xj(1), xj_th = xj(2);

        // Rotation of node i
        Eigen::Matrix2d Ri;
        Ri << cos(xi_th),  sin(xi_th),
            -sin(xi_th),  cos(xi_th);

        
        // compute jacobian
        GN::Mat36 J;
        J.setZero();

        // d e_trans / d xi 
        J.block<2,2>(0,0) = -Ri;

        // d e_trans / d xj
        J.block<2,2>(0,3) = Ri;

        // d e_trans / d theta_i
        Eigen::Vector2d temp(-(xj_x - xi_x) * sin(xi_th) - (xj_y - xi_y) * cos(xi_th),
                            (xj_x - xi_x) * cos(xi_th) - (xj_y - xi_y) * sin(xi_th));

        Eigen::Vector2d dtrans_dtheta_i = Ri * temp;
        J(0,2) = dtrans_dtheta_i(0);
        J(1,2) = dtrans_dtheta_i(1);

        // d e_theta / d xi
        J(2,2) = -1;

        // d e_theta / d xj
        J(2,5) = 1;

        return {e, J};
    }

    bool GnOptimizer::buildLinearHb(const size_t n,
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
        */

        for (size_t k = 0; k < n; k++)
        {
            int i = k+1;
            int j = k;

            // Set poses as vectors
            GN::Vec3 xi(X_[i].pose.x, X_[i].pose.y, X_[i].pose.theta);
            GN::Vec3 xj(X_[j].pose.x, X_[j].pose.y, X_[j].pose.theta);
            
            // Get the error vec and Jacobian mat
            auto [e, J] = computeErrorAndJacobian(xi, xj, Z_[k]);

            // Set a threshold for error vec (euclidean distance)
            if (e.norm() < config_.threshold)
            {
                std::cout << "OPTIMIZATION MET THRESHOLD" << std::endl;
                return true;
            }

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

            return false;
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

        // Allocate space to copy nodes in reverse order
        // We don't want to edit the nodes diretly
        // std::vector<utils::Node> X = N_;
        // std::reverse(X.begin(), X.end());

        for (size_t j = 0; j < config_.max_iters; j++)
        {
            // Intialize empty matrix and vector
            GN::dMat H = GN::dMat::Zero(3*n, 3*n);
            GN::dVec b = GN::dVec::Zero(3*n);
            
            // Extract the H adjancency matrix and b coefficient vector
            bool met_threshold = buildLinearHb(n, H, b);

            // Checking for convergence
            if (!met_threshold)
            {
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
                ////////////////////////////////////
            
                // Update state vectors
                for (size_t i = 0; i < X_.size(); i++)
                {
                    X_[i].pose.x += dX.segment<3>(3*i)(0);
                    X_[i].pose.y += dX.segment<3>(3*i)(1);
                    X_[i].pose.theta += dX.segment<3>(3*i)(2);
                }
            } else {
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



