#include "capybara/checkers.hpp"

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




}

