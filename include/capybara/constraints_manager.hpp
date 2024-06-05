#pragma once
#include <capybara/capytypes.hpp>
#include <capybara/numpy.hpp>


#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

using namespace Eigen;

namespace Capybara {


class ConstraintsManager
{
protected:
    int dim_configuration_;
    MatrixXd equality_constraint_matrix_ = MatrixXd::Zero(0,0);
    VectorXd equality_constraint_vector_ = VectorXd::Zero(0);
    MatrixXd inequality_constraint_matrix_ = MatrixXd::Zero(0,0);
    VectorXd inequality_constraint_vector_ = VectorXd::Zero(0);

    void _reset_equality_constraints();
    void _reset_inequality_constraints();

public:
    ConstraintsManager() = delete;
    ConstraintsManager(const int& dim_configuration);

    void add_equality_constraint(const MatrixXd& Aeq, const VectorXd& beq);
    void add_inequality_constraint(const MatrixXd& A, const VectorXd& b);

    std::tuple<MatrixXd, VectorXd> get_equality_constraints(const bool& delete_equality_constraints = true);
    std::tuple<MatrixXd, VectorXd> get_inequality_constraints(const bool& delete_inequality_constraints = true);


};

}

