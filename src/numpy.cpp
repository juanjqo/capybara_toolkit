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

#include <capybara/numpy.hpp>

namespace Capybara {

/**
 * @brief Numpy::vstack stacks matrices in sequence vertically
 * @param A
 * @param B
 * @return The matrix [A;
 *                     B]
 */
MatrixXd Numpy::vstack(const MatrixXd &A, const MatrixXd &B)
{
    Checkers::check_column_matrix_sizes(A,B);
    int m_A = A.rows();
    int m_B = B.rows();
    int n_A = A.cols();
    int n_B = B.cols();
    MatrixXd C = MatrixXd::Zero(m_A + m_B, n_A);
    C.block(0,0, m_A, n_A) = A;
    C.block(m_A, 0, m_B, n_B) = B;
    return C;
}


/**
 * @brief Numpy::hstack stacks matrices in sequence horizontally
 * @param A
 * @param B
 * @return The matrix [A B]
 */
MatrixXd Numpy::hstack(const MatrixXd &A, const MatrixXd &B)
{
    Checkers::check_row_matrix_sizes(A,B);
    int m_A = A.rows();
    int m_B = B.rows();
    int n_A = A.cols();
    int n_B = B.cols();
    MatrixXd C = MatrixXd::Zero(m_A, n_A + n_B);
    C.block(0,0, m_A, n_A) = A;
    C.block(0, n_A, m_B, n_B) = B;
    return C;
}


/**
 * @brief Numpy::block_diag creates a diagonal matrix
 * @param A The vector of matrices;
 * @return The diagonal matrix composed of elements of A.
 */
MatrixXd Numpy::block_diag(const std::vector<MatrixXd> &A)
{
    int m = 0;
    int n = 0;
    for(auto& matrix : A)
    {
        m = m + matrix.rows();
        n = n + matrix.cols();
    }
    MatrixXd J = MatrixXd::Zero(m, n);
    int im=0;
    int in=0;
    for(auto& matrix : A)
    {
        m = matrix.rows();
        n = matrix.cols();
        J.block(im,in, m,n) = matrix;
        im = im + m;
        in = in + n;
    }
    return J;
}


/**
 * @brief Numpy::block_diag creates a diagonal matrix
 * @param A The vector of doubles.
 * @return The diagonal matrix composed of elements of A.
 */
MatrixXd Numpy::block_diag(const std::vector<double> &A)
{
    std::vector<double> vec = A;
    return Capybara::Conversions::std_vector_double_to_vectorxd(vec).asDiagonal();
}


/**
 * @brief Numpy::resize resizes a matrix A to a larger matrix of size (rowsxcols) that containts
 *               the matrix A. The additional elements are zeros.
 * @param A
 * @param rows
 * @param cols
 * @return The matrix [A 0
 *                     0 0]
 */
MatrixXd Numpy::resize(const MatrixXd &A, const int &rows, const int &cols)
{
    MatrixXd aux = MatrixXd::Zero(rows, cols);
    int m = A.rows();
    int n = A.cols();

    if (m > rows)
    {
        throw std::runtime_error(std::string("The rows you used is smaller than the rows of the Matrix. ")
                                 +std::string("Incompatible rows for resize. Matrix A has ")
                                 +std::to_string(m)+ std::string(" rows. But you used ")
                                 +std::to_string(rows));
    }
    if (n > cols)
    {
        throw std::runtime_error(std::string("The cols you used is smaller than the cols of the Matrix. ")
                                 +std::string("Incompatible cols for resize. Matrix A has ")
                                 +std::to_string(n)+ std::string(" cols. But you used ")
                                 +std::to_string(cols));
    }

    aux.block(0,0, m, n) = A;
    return aux;

}

/**
 * @brief Numpy::linspace
 * @param start
 * @param stop
 * @param size
 * @return
 */
VectorXd Numpy::linspace(const double &start, const double &stop, const int &size)
{
    return VectorXd::LinSpaced(size, start, stop);
}


/**
 * @brief Numpy::linspace
 * @param start
 * @param stop
 * @param size
 * @return
 */
MatrixXd Numpy::linspace(const VectorXd &start, const VectorXd &stop, const int &size)
{
    if (start.size() != stop.size())
    {
        throw std::runtime_error(std::string("Both vectors must have the same dimension. ")
                                 +std::string("You used vectors of size ")
                                 +std::to_string(start.size())+ std::string(" and ")
                                 +std::to_string(stop.size()));
    }
    MatrixXd result = MatrixXd::Zero(start.size(), size);
    for(int i=0; i < start.size(); ++i)
    {
        result.row(i) = Numpy::linspace(start[i], stop[i], size);
    }
    return result;

}

MatrixXd Numpy::symmetric(const MatrixXd &J)
{
    return J.transpose()*J;
}

MatrixXd Numpy::symmetric(const MatrixXd &J, const double &damping)
{
    return J.transpose()*J + damping*MatrixXd::Identity(J.cols(),J.cols());
}

/**
 * @brief Numpy::symmetric_and_linear_component given the quadratic program
 *
 *          min(x) ||J*x + gain*task_error|| + damping*||x||,
 *
 *          this method returns the tuple {H, f},
 *          where H = J.T*J + damping*Identity, and f = gain*J.transpose()*task_error.
 *
 *
 * @param J
 * @param damping
 * @param gain
 * @param task_error
 * @return
 */
std::tuple<MatrixXd, VectorXd> Numpy::symmetric_and_linear_component(const MatrixXd& J,
                                                                     const double &damping,
                                                                     const double& gain,
                                                                     const VectorXd& task_error)
{
    return {Capybara::Numpy::symmetric(J, damping), gain*J.transpose()*task_error};
}

}
