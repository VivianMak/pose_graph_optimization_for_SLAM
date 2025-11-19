#include <ctime>
#include <iostream>

#include "gn_optimizer.hpp"

// Gauss Newton solver
#ifndef SOLVER
#define SOLVER Eigen::SimplicialCholesky<SpMat>
#endif

namespace gn
{
    // Runtime debugger
    std::clock_t start;
    start = std::clock();

    /*
    * given the wheel odometry transform matrix (xi xj),
    * then the ICP transform (Zij)
    * multiply thest two 3x3 matricies to get a vector of 3 by 1
    */

    std::cout << "Gauss-Newton Complete!" << std::endl;
    return true;
    
}