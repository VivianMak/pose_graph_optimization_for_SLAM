#include <functional>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>

// prevents from including twice
#ifndef GN_H
#define GN_H

namespace GN{
    class GN{
    public:
        // using?
        typedef Eigen::Matrix<double,3,1> Vec3;     // error vectors
        typedef Eigen::Matrix<double,3,3> Mat33;    // info matrix, transforms
        typedef Eigen::Matrix<double,3,6> Mat36;    // jacobians

        typedef Eigen::VectorXd dVec; // for b
        typedef Eigen::MatrixXd dMat; // for H

        // class intialization with matricies from ICP (Z) and node vector
        GN(const std::vector<Vec3> &Z, const std::vector<Vec3> &N, const GN_Config config);

        struct GN_Config
        {
            int max_iters = 10;
            Vec3 threshold = 0.5, 0.5, 1;   // x, y, theta (radians)
            Mat33 omega = = Eigen::Matrix3d::Identity();
        }

        struct eJ{
            Vec3 e;
            Mat36 J;
        }

        // main function to run
        bool gnOptimizer();
                        // const std::vector Z, 
                        // cosnt std::vector<utils::Node> N, 
                        // GN_Config config

        // HELPER FUNCTIONS

        // compute error and Jacobian for a single edge
        GN::eJ computeErrorAndJacobian(const GN::Vec3 &xi, 
                                        const GN::Vec3 &xj, 
                                        const GN::Mat33 &z_ij);

        // // compute Jacobians for all edges
        // void linearize(std::vector<Mat36> &J_out,
        //             std::vector<Vec3> &errors_out) const;

        // Build global H and b for all edges
        void buildLinearHb(const size_t n,
                        const std::vector<Vec3> &X,  
                        // const std::vector<Mat33> &Z,
                        GN::dMat &H,
                        GN::dVec &b);

    private:
        std::vector<Vec3> Z_;  // icp transform list
        std::vector<Vec3> N_;  // node poses list
        GN_Config config_;

    }
}
#endif // GN_H

    

    
