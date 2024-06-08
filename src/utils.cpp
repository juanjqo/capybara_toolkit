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

#include <capybara/utils.hpp>

/**
 * @brief Capybara::CVectorXd
 * @param vec
 * @return
 */
VectorXd Capybara::CVectorXd(const std::vector<double> &vec)
{
    std::vector<double> myvec = vec;
    return Capybara::Conversions::std_vector_double_to_vectorxd(myvec);
}

/**
 * @brief Capybara::CMatrixXd
 * @param mat
 * @return
 */
MatrixXd Capybara::CMatrixXd(const std::vector<std::vector<double>>& mat)
{
    int rows = mat.size();
    std::vector<int> sizes(rows, 0);
    for (auto i=0;i<rows;i++)
        sizes.at(i) = mat.at(i).size();

    if (!Capybara::Checkers::check_equal_elements(sizes, Capybara::Checkers::MODE::DO_NOT_PANIC))
        throw std::runtime_error("Panic with Capybara::MatrixXd(). Wrong number of elements. "
                                 "All vector must have the same dimension.  ");

    int cols = sizes.at(0);
    MatrixXd output = MatrixXd(rows,cols);
    auto mymat = mat;
    for (auto i=0;i<rows;i++)
        output.row(i) = Capybara::Conversions::std_vector_double_to_vectorxd(mymat.at(i));

    return output;
}

void Capybara::delay(const int &milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}
