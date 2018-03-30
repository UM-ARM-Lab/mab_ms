#include "smmap/gurobi_solvers.h"
#include <gurobi_c++.h>
#include <iostream>
#include <mutex>
#include <Eigen/Eigenvalues>

using namespace Eigen;

static std::mutex gurobi_env_construct_mtx;

GRBQuadExpr normSquared(const std::vector<GRBLinExpr>& exprs)
{
    GRBQuadExpr vector_norm_squared = 0;

    // TODO: replace with a single call to addTerms?
    for (size_t expr_ind = 0; expr_ind < exprs.size(); expr_ind++)
    {
        vector_norm_squared += exprs[expr_ind] * exprs[expr_ind];
    }

    return vector_norm_squared;
}

GRBQuadExpr normSquared(const std::vector<GRBLinExpr>& exprs, const VectorXd& weights)
{
    assert(exprs.size() == (size_t)weights.rows());
    GRBQuadExpr vector_norm_squared = 0;

    // TODO: replace with a single call to addTerms?
    for (size_t expr_ind = 0; expr_ind < exprs.size(); expr_ind++)
    {
        vector_norm_squared += weights((ssize_t)expr_ind) * exprs[expr_ind] * exprs[expr_ind];
    }

    return vector_norm_squared;
}

GRBQuadExpr normSquared(GRBVar* vars, const ssize_t num_vars)
{
    GRBQuadExpr vector_norm_squared = 0;

    // TODO: replace with a single call to addTerms?
    for (ssize_t var_ind = 0; var_ind < num_vars; var_ind++)
    {
        vector_norm_squared += vars[var_ind] * vars[var_ind];
    }

    return vector_norm_squared;
}

std::vector<GRBLinExpr> buildVectorOfExperssions(const MatrixXd& A, GRBVar* vars, const VectorXd& b)
{
    const ssize_t num_expr = A.rows();
    const ssize_t num_vars = A.cols();
    std::vector<GRBLinExpr> exprs(num_expr, 0);

    for (ssize_t expr_ind = 0; expr_ind < num_expr; expr_ind++)
    {
        for (ssize_t var_ind = 0; var_ind < num_vars; var_ind++)
        {
            exprs[expr_ind] += A(expr_ind, var_ind) * vars[var_ind];
        }
        exprs[expr_ind] -= b(expr_ind);
    }

    return exprs;
}

VectorXd smmap::minSquaredNorm(const MatrixXd& A, const VectorXd& b, const double max_x_norm)
{
    VectorXd x;
    GRBVar* vars = nullptr;
    try
    {
        const ssize_t num_vars = A.cols();
        const std::vector<double> lb(num_vars, -max_x_norm);
        const std::vector<double> ub(num_vars, max_x_norm);

        // TODO: Find a way to put a scoped lock here
        gurobi_env_construct_mtx.lock();
        GRBEnv env;
        gurobi_env_construct_mtx.unlock();

        env.set(GRB_IntParam_OutputFlag, 0);
        GRBModel model(env);
        vars = model.addVars(lb.data(), ub.data(), nullptr, nullptr, nullptr, (int)num_vars);
        model.update();

        model.addQConstr(normSquared(vars, num_vars), GRB_LESS_EQUAL, max_x_norm * max_x_norm);
        model.setObjective(normSquared(buildVectorOfExperssions(A, vars, b)), GRB_MINIMIZE);
        model.update();
        model.optimize();

        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
        {
            x.resize(num_vars);
            for (ssize_t var_ind = 0; var_ind < num_vars; var_ind++)
            {
                x(var_ind) = vars[var_ind].get(GRB_DoubleAttr_X);
            }
        }
        else
        {
            std::cout << "Status: " << model.get(GRB_IntAttr_Status) << std::endl;
            exit(-1);
        }
    }
    catch(GRBException e)
    {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    }
    catch(...)
    {
        std::cout << "Exception during optimization" << std::endl;
    }

    delete[] vars;
    return x;
}

VectorXd smmap::minSquaredNorm(const MatrixXd& A, const VectorXd& b, const double max_x_norm, const VectorXd& weights)
{
    VectorXd x;
    GRBVar* vars = nullptr;
    try
    {
        const ssize_t num_vars = A.cols();
        const std::vector<double> lb(num_vars, -max_x_norm);
        const std::vector<double> ub(num_vars, max_x_norm);

        // TODO: Find a way to put a scoped lock here
        gurobi_env_construct_mtx.lock();
        GRBEnv env;
        gurobi_env_construct_mtx.unlock();

        env.set(GRB_IntParam_OutputFlag, 0);
        GRBModel model(env);
        vars = model.addVars(lb.data(), ub.data(), nullptr, nullptr, nullptr, (int)num_vars);
        model.update();

        model.addQConstr(normSquared(vars, num_vars), GRB_LESS_EQUAL, max_x_norm * max_x_norm);

        GRBQuadExpr objective_fn = normSquared(buildVectorOfExperssions(A, vars, b), weights);
        // Check if we need to add anything extra to the main diagonal.
        const VectorXd eigenvalues = (A.transpose() * weights.asDiagonal() * A).selfadjointView<Upper>().eigenvalues();
        if ((eigenvalues.array() < 1.1e-4).any())
        {
            const std::vector<double> diagonal(num_vars, 1.1e-4 - eigenvalues.minCoeff());
            objective_fn.addTerms(diagonal.data(), vars, vars, (int)num_vars);
        }
        model.setObjective(objective_fn, GRB_MINIMIZE);

        model.update();
        model.optimize();

        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
        {
            x.resize(num_vars);
            for (ssize_t var_ind = 0; var_ind < num_vars; var_ind++)
            {
                x(var_ind) = vars[var_ind].get(GRB_DoubleAttr_X);
            }
        }
        else
        {
            std::cout << "Status: " << model.get(GRB_IntAttr_Status) << std::endl;
            exit(-1);
        }
    }
    catch(GRBException e)
    {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    }
    catch(...)
    {
        std::cout << "Exception during optimization" << std::endl;
    }

    delete[] vars;
    return x;
}

Eigen::VectorXd smmap::minSquaredNormSE3VelocityConstraints(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, const double max_se3_velocity, const Eigen::VectorXd& weights)
{
    VectorXd x;
    GRBVar* vars = nullptr;
    try
    {
        const ssize_t num_vars = A.cols();
        assert(num_vars % 6 == 0);

        const std::vector<double> lb(num_vars, -max_se3_velocity);
        const std::vector<double> ub(num_vars, max_se3_velocity);

        // TODO: Find a way to put a scoped lock here
        gurobi_env_construct_mtx.lock();
        GRBEnv env;
        gurobi_env_construct_mtx.unlock();

        env.set(GRB_IntParam_OutputFlag, 0);
        GRBModel model(env);
        vars = model.addVars(lb.data(), ub.data(), nullptr, nullptr, nullptr, (int)num_vars);
        model.update();

        // Add the SE3 velocity constraints
        for (int i = 0; i < num_vars / 6; i++)
        {
            model.addQConstr(normSquared(&vars[i * 6], 6), GRB_LESS_EQUAL, max_se3_velocity * max_se3_velocity);
        }

        GRBQuadExpr objective_fn = normSquared(buildVectorOfExperssions(A, vars, b), weights);
        // Check if we need to add anything extra to the main diagonal.
        const VectorXd eigenvalues = (A.transpose() * weights.asDiagonal() * A).selfadjointView<Upper>().eigenvalues();
        if ((eigenvalues.array() < 1.1e-4).any())
        {
            std::vector<double> diagonal(num_vars, 1.1e-4 - eigenvalues.minCoeff());
            objective_fn.addTerms(diagonal.data(), vars, vars, (int)num_vars);
        }
        model.setObjective(objective_fn, GRB_MINIMIZE);

        model.update();
        model.optimize();

        if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL)
        {
            x.resize(num_vars);
            for (ssize_t var_ind = 0; var_ind < num_vars; var_ind++)
            {
                x(var_ind) = vars[var_ind].get(GRB_DoubleAttr_X);
            }
        }
        else
        {
            std::cout << "Status: " << model.get(GRB_IntAttr_Status) << std::endl;
            exit(-1);
        }
    }
    catch(GRBException e)
    {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    }
    catch(...)
    {
        std::cout << "Exception during optimization" << std::endl;
    }

    delete[] vars;
    return x;
}
