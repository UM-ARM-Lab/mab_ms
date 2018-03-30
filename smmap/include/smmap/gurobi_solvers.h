#ifndef GUROBI_SOLVERS_H
#define GUROBI_SOLVERS_H

#include <Eigen/Dense>

namespace smmap
{
    Eigen::VectorXd minSquaredNorm(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const double max_x_norm);
    Eigen::VectorXd minSquaredNorm(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const double max_x_norm, const Eigen::VectorXd& weights);
    Eigen::VectorXd minSquaredNormSE3VelocityConstraints(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const double max_se3_velocity, const Eigen::VectorXd& weights);
}

#endif // GUROBI_SOLVERS_H
