#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "gn_optimizer.hpp"
#include "gn_helper.hpp"

namespace GN
{
    GN::eJ computeErrorAndJacobian(const GN::Vec3 &xi, 
                                    const GN::Vec3 &xj, 
                                    const GN::Mat33 &z_ij)
    {
        /*
        * Compute the error and Jacobian for adjacent nodes i,j
        *
        * @param xi - ith pose
        * @param xj - jth pose
        * @param zij - icp transform btwn ij
        * 
        * @return eJ - error vec and Jacobian mat
        */

        // initalize error and jacobian for current adjancent nodes
        GN::eJ currenteJ;

        // compute error
        currenteJ.e = gn_helper::transforms_to_dvector(xi, xj. zij);
        
        // compute jacobian
        GN::Mat36 J;
        J.setZero();

        // decompose poses, use Pose struct?
        double xi_x = xi(0), xi_y = xi(1), xi_th = xi(2);
        double xj_x = xj(0), xj_y = xj(1), xj_th = xj(2);

        // Rotation of node i
        Eigen::Matrix2d Ri;
        Ri << cos(xi_th),  sin(xi_th),
            -sin(xi_th),  cos(xi_th);

        // BRUH WTF
        // d e_trans / d xi (2x2 block)
        J.block<2,2>(0,0) = -Ri;

        // d e_trans / d theta_i
        Eigen::Vector2d temp(-(xj_x - xi_x) * sin(xi_th) - (xj_y - xi_y) * cos(xi_th),
                            (xj_x - xi_x) * cos(xi_th) - (xj_y - xi_y) * sin(xi_th));

        Eigen::Vector2d dtrans_dtheta_i = Ri * temp;
        J(0,2) = dtrans_dtheta_i(0);
        J(1,2) = dtrans_dtheta_i(1);

        // d e_theta / d xi
        J(2,2) = -1;

        // d e_trans / d xj
        J.block<2,2>(0,3) = Ri;

        // d e_theta / d xj
        J(2,5) = 1;

        currenteJ.J = J;

        return GN::eJ currenteJ;
    }

    // void linearize()
    // {

    // }

    void buildLinearHb(const size_t n,
                        const std::vector<GN::Vec3> &X,  
                        const std::vector<GN::Mat33> &Z,
                        GN::dMat &H,
                        GN::dVec &b)
    {
        /*
        * Build the coeffient H matrix and b vector
        *
        * @param n - length
        * @param X - vector of poses copied from nodes (N) vector
        * @param Z - vector of 3x3 homogeneous matrix from ICP
        * @param H - empty size-defined sparse matrix to build
        * @param b - empty size-defined vector to build
        */

        for (size_t k = 0; k < n; k++){
            int i = k;
            int j = k+1;
            
            // Get the error vec and Jacobian mat
            GN::eJ eJ = computeErrorAndJacobian(X[i], X[j], Z[k]);

            GN::Vec3 e = eJ.e;
            GN::Mat36 J = eJ.J;

            Eigen::Matrix<double,6,6> H_ij = J.transpose() * J;
            Eigen::Matrix<double,6,1> b_ij = -J.transpose() * e;

            // Fill into global H and b
            H.block<3,3>(3*i, 3*i) += H_ij.block<3,3>(0,0);
            H.block<3,3>(3*i, 3*j) += H_ij.block<3,3>(0,3);
            H.block<3,3>(3*j, 3*i) += H_ij.block<3,3>(3,0);
            H.block<3,3>(3*j, 3*j) += H_ij.block<3,3>(3,3);

            b.segment<3>(3*i) += b_ij.segment<3>(0);
            b.segment<3>(3*j) += b_ij.segment<3>(3);
            }
        

    }

    bool gnOptimizer(const std::vector Z, 
                    cosnt std::vector<utils::Node> N, 
                    GN_Config config)
    {
        /*
        * [main] Run the gauss newton global optimizer for n iterations
        *
        * @param Z - vector of 3x3 homogeneous matrix from ICP
        * @param N - vector of nodes, will extract 3x3 homogeneous matrix from eddge odom
        * @param GN_Config
        * 
        * @return bool - true/false for if optimizer has finished (threshold?)
        */

        // Runtime debugger
        std::clock_t start;
        start = std::clock();

        std::cout << "Running Gauss-Newton Optimization..."

        size_t n = Z.size();

        // intialize variables and size
        H = GN::dMat::Zero(3*N, 3*N);
        b = GN::dVec::Zero(3*N);


        // we dont want to edit the nodes directly
        // make a copy to perform the optimization
        // visualize: lay X over N to show changes.
        
        /*
        * TODO:
        * - smart pointers
        * - invert X for it to go backwards
        * - make it a vector of poses using <utils::Poses>
        */
        std::vector<GN::Vec3> X = N.poses;

        // GN::dMat Jacobian;
        
        // get the H and b coefficient matricies
        buildLinearHb(n, X, Z, H, b);

        // solving for delta x (x,y,theta)
        GN::Vec3 dX = H.ldlt().solve(b);
        

        // SET THRESHOLD OR ITERATIONS

        // update state vectors
        for (size_t i = 0; i < X.size(); i++)
        {
            X[i] += dx.segment<3>(3*i);
        }

        

        // Finished
        double duration = (std::clock() - start) / (double)CLOCKS_PER_SEC;
        std::cout << "Gauss-Newton Optimization Complete!" << std::endl;
        std::cout << std::format("Optimization took %gsecs.\n", duration);
        return true;

        // return false somewhere

    }

    
}



