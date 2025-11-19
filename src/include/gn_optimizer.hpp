#include <functional>
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>


namespace gn{

    typedef Eigen::VectorXd                 dVec;
    typedef Eigen::SparseMatrix<double>     SpMat;
    typedef SpMat::InnerIterator            Iter;
}