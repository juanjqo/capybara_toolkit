/*
#    Copyright (c) 2024 Juan Jose Quiroz Omana
#
#    Capybara_toolkit is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    Capybara_toolkit is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with Capybara_toolkit.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Juan Jose Quiroz Omana, (email: juanjqogm@gmail.com)
#
# ################################################################
*/

#include <capybara/constraints_manager.hpp>

namespace Capybara {


ConstraintsManager::ConstraintsManager(const int &dim_configuration)
:dim_configuration_(dim_configuration)
{

}

void ConstraintsManager::_reset_equality_constraints()
{
    equality_constraint_matrix_ = MatrixXd::Zero(0,0);
    equality_constraint_vector_ = VectorXd::Zero(0);
}

void ConstraintsManager::_reset_inequality_constraints()
{
    inequality_constraint_matrix_ = MatrixXd::Zero(0,0);
    inequality_constraint_vector_ = VectorXd::Zero(0);
}

/**
 * @brief ConstraintsManager::add_equality_constraint adds the equality constraint Aeq*x = beq
 *        to the set of equality constraints.
 *
 * @param Aeq
 * @param beq
 */
void ConstraintsManager::add_equality_constraint(const MatrixXd &Aeq, const VectorXd &beq)
{
    Capybara::Checkers::check_constraint_sizes(Aeq,beq, dim_configuration_);

    if (equality_constraint_matrix_.size() == 0)
    {
        equality_constraint_matrix_ = Capybara::Numpy::resize(Aeq, Aeq.rows(), dim_configuration_);
        equality_constraint_vector_ = beq;
    }else
    {
        equality_constraint_matrix_ = Capybara::Numpy::vstack(equality_constraint_matrix_, Aeq);
        equality_constraint_vector_ = Capybara::Numpy::vstack(equality_constraint_vector_, beq);
    }

}

/**
 * @brief ConstraintsManager::add_inequality_constraint adds the inequality constraint A*x <= b
 *        to the set of inequality constraints.
 *
 * @param A
 * @param b
 */
void ConstraintsManager::add_inequality_constraint(const MatrixXd &A, const VectorXd &b)
{

    Capybara::Checkers::check_constraint_sizes(A,b, dim_configuration_);

    if (inequality_constraint_matrix_.size() == 0)
    {
        inequality_constraint_matrix_ = Capybara::Numpy::resize(A, A.rows(), dim_configuration_);
        inequality_constraint_vector_ = b;
    }else
    {
        inequality_constraint_matrix_ = Capybara::Numpy::vstack(inequality_constraint_matrix_, A);
        inequality_constraint_vector_ = Capybara::Numpy::vstack(inequality_constraint_vector_, b);
    }

}

/**
 * @brief ConstraintsManager::get_equality_constraints returns the set of equality constraints
 *        composed of the Matrix A and the vector b, where
 *        A*x = b.
 *
 * @param delete_equality_constraints Flag used to delete all the equality constraints stored. (Default)
 * @return {Aeq, beq}
 */
std::tuple<MatrixXd, VectorXd> ConstraintsManager::get_equality_constraints(const bool &delete_equality_constraints)
{
    auto Aeq = equality_constraint_matrix_;
    auto beq = equality_constraint_vector_;
    if (delete_equality_constraints)
        _reset_equality_constraints();
    return {Aeq, beq};
}


/**
 * @brief ConstraintsManager::get_inequality_constraints returns the set of inequality constraints
 *        composed of the Matrix A and the vector b, where
 *        A*x <= b
 *
 * @param delete_inequality_constraints Flag used to delete all the inequality constraints stored. (Default)
 * @return
 */
std::tuple<MatrixXd, VectorXd> ConstraintsManager::get_inequality_constraints(const bool &delete_inequality_constraints)
{
    auto A = inequality_constraint_matrix_;
    auto b = inequality_constraint_vector_;
    if (delete_inequality_constraints)
        _reset_inequality_constraints();
    return {A, b};
}





}
