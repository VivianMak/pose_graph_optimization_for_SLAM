#include <ctime>
#include <iostream>

#include "gn_optimizer.hpp"

SE2Edge::SE2Edge(const std::vector<Vec3> &Z,
                 const std::vector<Vec3> &N)
    : Z_(Z), N_(N)
{
    // initialize information matrix with alkdfjaslfd;
    Omega_.setIdentity();
}

SE2Edge::Vec3 SE2Edge::errorAndJacobian(
        const Vec6 &x,
        const Vec3 &z,
        Mat36 &J)
{
    double xi = x(0), yi = x(1), thi = x(2);
    double xj = x(3), yj = x(4), thj = x(5);

    Eigen::Matrix2d Ri;
    Ri <<  cos(thi),  sin(thi),
          -sin(thi),  cos(thi);

    Eigen::Vector2d tij(xj - xi, yj - yi);

    Eigen::Vector2d e_trans = Ri * tij - z.head<2>();
    double e_theta = (thj - thi) - z(2);

    Vec3 e;
    e << e_trans(0), e_trans(1), e_theta;

    // Build Jacobian
    J.setZero();

    // d e_trans / d pose i
    J.block<2,2>(0,0) = -Ri;

    Eigen::Vector2d temp(-(xj-xi)*sin(thi) - (yj-yi)*cos(thi),
                          (xj-xi)*cos(thi) - (yj-yi)*sin(thi));

    Eigen::Vector2d dtrans_dtheta_i = Ri * temp;

    J(0,2) = dtrans_dtheta_i(0);
    J(1,2) = dtrans_dtheta_i(1);
    J(2,2) = -1;

    // d e_trans / d pose j
    J.block<2,2>(0,3) = Ri;
    J(2,5) = 1;

    return e;
}

void SE2Edge::computeErrors(std::vector<Vec3> &errors,
                            std::vector<double> &stateCost) const
{
    errors.clear();
    stateCost.clear();

    for (size_t i = 0; i < Z_.size(); i++)
    {
        Vec3 xi = N_[i];
        Vec3 xj = N_[i+1];
        Vec3 z  = Z_[i];

        Vec6 x;
        x << xi(0), xi(1), xi(2),
             xj(0), xj(1), xj(2);

        Mat36 J;
        Vec3 e = errorAndJacobian(x, z, J);

        errors.push_back(e);

        double c = e.transpose() * Omega_ * e;
        stateCost.push_back(c);
    }
}

void SE2Edge::linearize(std::vector<Mat36> &J_out,
                        std::vector<Vec3> &errors_out) const
{
    J_out.clear();
    errors_out.clear();

    for (size_t i = 0; i < Z_.size(); i++)
    {
        Vec3 xi = N_[i];
        Vec3 xj = N_[i+1];
        Vec3 z  = Z_[i];

        Vec6 x;
        x << xi(0), xi(1), xi(2),
             xj(0), xj(1), xj(2);

        Mat36 J;
        Vec3 e = errorAndJacobian(x, z, J);

        errors_out.push_back(e);
        J_out.push_back(J);
    }
}

// -----------------------------------------------------------
// BUILD GLOBAL H AND b
// -----------------------------------------------------------
void SE2Edge::buildHAndb(Eigen::MatrixXd &H,
                         Eigen::VectorXd &b) const
{
    size_t M = Z_.size();
    H = Eigen::MatrixXd::Zero(6*M, 6*M);
    b = Eigen::VectorXd::Zero(6*M);

    std::vector<Mat36> J_list;
    std::vector<Vec3> e_list;
    linearize(J_list, e_list);

    for (size_t i = 0; i < M; i++)
    {
        const Mat36 &Ji = J_list[i];
        const Vec3  &ei = e_list[i];

        Eigen::Matrix<double,6,6> Hi = Ji.transpose() * Omega_ * Ji;
        Eigen::Matrix<double,6,1> bi = -Ji.transpose() * Omega_ * ei;

        H.block<6,6>(6*i, 6*i) = Hi;
        b.segment<6>(6*i)      = bi;
    }
}





#include <iostream>
#include <vector>
#include <Eigen/Dense>

using Vec3 = Eigen::Vector3d;
using Mat36 = Eigen::Matrix<double,3,6>;
using Mat66 = Eigen::Matrix<double,6,6>;

// ------------------------------------------------------------
// Compute error and Jacobian for ONE edge (i → j)
// ------------------------------------------------------------
Vec3 errorAndJacobian(const Vec3 &xi, const Vec3 &xj, 
                      const Vec3 &z_ij, Mat36 &J)
{
    double xi_x = xi(0), xi_y = xi(1), xi_th = xi(2);
    double xj_x = xj(0), xj_y = xj(1), xj_th = xj(2);

    // Rotation of node i
    Eigen::Matrix2d Ri;
    Ri << cos(xi_th),  sin(xi_th),
         -sin(xi_th),  cos(xi_th);

    Eigen::Vector2d tij(xj_x - xi_x, xj_y - xi_y);

    // Error = predicted_relative - measured_relative
    Eigen::Vector2d e_trans = Ri * tij - z_ij.head<2>();
    double e_theta = (xj_th - xi_th) - z_ij(2);

    Vec3 e;
    e << e_trans, e_theta;

    // -------------------- JACOBIAN --------------------
    J.setZero();

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

    return e;
}

// ------------------------------------------------------------
// Build global H and b for full pose graph
// ------------------------------------------------------------
void buildSystem(const std::vector<Vec3> &X,
                 const std::vector<Vec3> &Z,
                 Eigen::MatrixXd &H,
                 Eigen::VectorXd &b)
{
    size_t N = X.size();
    size_t E = Z.size();  // edges = N-1 typically

    H = Eigen::MatrixXd::Zero(3*N, 3*N);
    b = Eigen::VectorXd::Zero(3*N);

    for (size_t k = 0; k < E; k++)
    {
        size_t i = k;
        size_t j = k+1;

        Mat36 J;
        Vec3 e = errorAndJacobian(X[i], X[j], Z[k], J);

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

    // Fix first pose (make it a prior)
    H.block<3,3>(0,0) += Eigen::Matrix3d::Identity() * 1e6;
}

// ------------------------------------------------------------
// ONE Gauss–Newton iteration
// ------------------------------------------------------------
std::vector<Vec3> gaussNewtonOptimize(const std::vector<Vec3> &N,
                                      const std::vector<Vec3> &Z,
                                      int iters = 10)
{
    std::vector<Vec3> X = N;  // copy initial guess

    for (int iter = 0; iter < iters; iter++)
    {
        Eigen::MatrixXd H;
        Eigen::VectorXd b;

        buildSystem(X, Z, H, b);

        Eigen::VectorXd dx = H.ldlt().solve(b);

        // update state
        for (size_t i = 0; i < X.size(); i++)
        {
            X[i] += dx.segment<3>(3*i);
        }

        double stepNorm = dx.norm();
        std::cout << "Iter " << iter
                  << "   step |dx| = " << stepNorm << std::endl;

        if (stepNorm < 1e-6)
            break;
    }

    return X;
}

// ------------------------------------------------------------
// MAIN: test the optimizer
// ------------------------------------------------------------
int main()
{
    // Initial poses N
    std::vector<Vec3> N = {
        Vec3(0.0, 0.0, 0.0),
        Vec3(1.2, -0.1, 0.05),
        Vec3(2.1, 0.4, 0.2)
    };

    // Measurements Z
    std::vector<Vec3> Z = {
        Vec3(1.0, 0.0, 0.0),    // meas from N0->N1
        Vec3(1.0, 0.0, 0.1)     // meas from N1->N2
    };

    std::cout << "Running Gauss-Newton...\n";

    std::vector<Vec3> Xopt = gaussNewtonOptimize(N, Z);

    std::cout << "\nOptimized poses:\n";
    for (const auto &p : Xopt)
        std::cout << p.transpose() << std::endl;

    return 0;
}
