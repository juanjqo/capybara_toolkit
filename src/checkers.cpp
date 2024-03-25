#include "checkers.h"
#include "capytypes.h"

namespace Capybara {

/**
 * @brief Checkers::check_column_matrix_sizes
 * @param A
 * @param B
 * @param mode
 * @return
 */
bool Checkers::check_column_matrix_sizes(const Eigen::MatrixXd &A,
                                         const Eigen::MatrixXd &B,
                                         const Checkers::MODE& mode)
{
    int n_A = A.cols();
    int n_B = B.cols();
    if (n_A != n_B)
    {
        if (mode == Checkers::MODE::PANIC )
            throw std::runtime_error(std::string("Incompatible sizes. The cols of Matrix A and B must have the same dimensions. ")
                                 + std::string("But A is ")+ std::to_string(A.rows())+ std::string("x")+ std::to_string(n_A)
                                 + std::string(" and B is ")+ std::to_string(B.rows()) + std::string("x")+ std::to_string(n_B));
        return FAIL;
    }
    return SUCCESS;

}

bool Checkers::check_row_matrix_sizes(const Eigen::MatrixXd &A,
                                      const Eigen::MatrixXd &B,
                                      const Checkers::MODE &mode)
{
    int m_A = A.rows();
    int m_B = B.rows();
    if (m_A != m_B)
    {
        if (mode == Checkers::MODE::PANIC )
            throw std::runtime_error(std::string("Incompatible sizes. The rows of Matrix A and B must have the same dimensions. ")
                                 + std::string("But A is ")+ std::to_string(m_A)+ std::string("x")+ std::to_string(A.cols())
                                 + std::string(" and B is ")+ std::to_string(m_B) + std::string("x")+ std::to_string(B.cols()));
        return FAIL;
    }
    return SUCCESS;

}

}

