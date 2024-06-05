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

#include <capybara/checkers.hpp>

namespace Capybara {

/**
 * @brief Checkers::check_column_matrix_sizes
 * @param A Matrix A
 * @param B Matrix B
 * @param mode PANIC if you want to throw a runtime error if
 *        matrices A and B have do not have the same number of columns.
 * @return True if matrices A and B have the same number of columns.
 *         False otherwise.
 */
bool Checkers::check_column_matrix_sizes(const MatrixXd &A,
                                         const MatrixXd &B,
                                         const Checkers::MODE& mode)
{
    int n_A = A.cols();
    int n_B = B.cols();
    if (n_A != n_B)
    {
        if (mode == Checkers::MODE::PANIC )
            throw std::runtime_error(std::string("Panic with Capybara::Checkers::check_column_matrix_sizes(A, B). ")
                                 + std::string("Incompatible sizes. The cols of Matrix A and B must have the same dimensions. ")
                                 + std::string("But A is ")+ std::to_string(A.rows())+ std::string("x")+ std::to_string(n_A)
                                 + std::string(" and B is ")+ std::to_string(B.rows()) + std::string("x")+ std::to_string(n_B));
        return FAIL;
    }
    return SUCCESS;

}


/**
 * @brief Checkers::check_row_matrix_sizes
 * @param A Matrix A
 * @param B Matrix B
 * @param mode PANIC if you want to throw a runtime error if
 *        matrices A and B have do not have the same number of rows.
 * @return True if matrices A and B have the same number of rows.
 *         False otherwise.
 */
bool Checkers::check_row_matrix_sizes(const MatrixXd &A,
                                      const MatrixXd &B,
                                      const Checkers::MODE &mode)
{
    int m_A = A.rows();
    int m_B = B.rows();
    if (m_A != m_B)
    {
        if (mode == Checkers::MODE::PANIC )
            throw std::runtime_error(std::string("Panic with Capybara::Checkers::check_row_matrix_sizes(A, B). ")
                                 + std::string("Incompatible sizes. The rows of Matrix A and B must have the same dimensions. ")
                                 + std::string("But A is ")+ std::to_string(m_A)+ std::string("x")+ std::to_string(A.cols())
                                 + std::string(" and B is ")+ std::to_string(m_B) + std::string("x")+ std::to_string(B.cols()));
        return FAIL;
    }
    return SUCCESS;

}


/**
 * @brief Checkers::check_constraint_sizes checks if the constraint set constraint A*x <= b or Ax = b
 *        have compatible sizes.
 * @param A
 * @param b
 * @param optimization_vector_size
 * @param mode
 * @return
 */
bool Checkers::check_constraint_sizes(const MatrixXd &A, const VectorXd &b, const double &optimization_vector_size, const MODE &mode)
{
    int m = A.rows();
    int n = A.cols();
    int nb = b.size();

    if ( n != optimization_vector_size)
    {
        if (mode == Checkers::MODE::PANIC )
            throw std::runtime_error(std::string("Panic with Capybara::Checkers::check_constraint_sizes(A, b, dim). ")
                                     + std::string("Incompatible sizes. The cols of Matrix A must have the same dimension of ")
                                     + std::string("the optimization vector, which is ") + std::to_string(optimization_vector_size)
                                     + std::string(". But the cols of A is ") + std::to_string(n));
        return FAIL;
    }
    if (m != nb)
    {
        if (mode == Checkers::MODE::PANIC )
            throw std::runtime_error(std::string("Panic with Capybara::Checkers::check_constraint_sizes(A, b, dim). ")
                                     + std::string("Incompatible sizes. The rows of Matrix A must have the same dimension of Vector b. ")
                                     + std::string("But you used A ")+ std::to_string(m)+ std::string("x")+ std::to_string(n)
                                     + std::string(" and b ")+ std::to_string(nb) + std::string("x1"));
        return FAIL;
    }

    return SUCCESS;
}





}

