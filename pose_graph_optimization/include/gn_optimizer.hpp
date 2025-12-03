#pragma once
#include <functional>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "utils.hpp"

// prevents from including twice
#ifndef GN_H
#define GN_H

namespace GN{

using Vec3 = Eigen::Vector3d; // error vectors
using Mat33 = Eigen::Matrix3d; // info matrix, transforms
using Mat36 = Eigen::Matrix<double,3,6>; // jacobians

using dVec = Eigen::VectorXd; // for b
using dMat = Eigen::MatrixXd; // for H

struct GN_Config{
    int max_iters;
    double threshold;  
    Mat33 omega; //Eigen::Matrix3d::Identity()
};

class GnOptimizer{
    public:
        
        // class intialization with matricies from ICP (Z) and node vector
        GnOptimizer(
            const std::vector<Mat33> &Z, 
            const std::vector<utils::Node> &N, 
            const GN_Config config);
        

        std::pair<GN::Vec3, GN::Mat36> 
        computeErrorAndJacobian(const Vec3 &xi, 
                                const Vec3 &xj, 
                                const Mat33 &z_ij);

        bool buildLinearHb(const size_t n,
                        const std::vector<utils::Node> &X,  
                        dMat &H,
                        dVec &b);

        // main function to run
        bool gnOptimizer();   

    private:
        std::vector<Mat33> Z_;  // icp transform list
        std::vector<utils::Node> N_;  // node poses list
        GN_Config config_;     // config parameters for optimization

    };
}
#endif // GN_H

    

    
