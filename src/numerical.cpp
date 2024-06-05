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

#include <capybara/numerical.hpp>

class Numerical
{

/**
 * @brief numerical_matrix_differentiation
 * @param J
 * @param T
 * @return
 */
std::vector<Eigen::MatrixXd> numerical_matrix_differentiation(const std::vector<MatrixXd> &J,
                                                              const double &T)
{
    int s = J.size();
    std::vector<MatrixXd> J_dot(s, MatrixXd::Zero(J[0].rows(), J[0].cols()));

    for(int i=4; i<s-4;i++) //for(int i=2; i<s-2;i++)
    {
        //J_dot[i] = ((J[i-2]-8*J[i-1]+8*J[i+1]-J[i+2])/(12*T));  // four-point central finite difference

        //Generation of finite difference formulas on arbitrarily spaced grids
        //Bengt Fornberg, 1988
        //10.1090/S0025-5718-1988-0935077-0
        J_dot[i] = (3*J[i-4]- 32*J[i-3]+168*J[i-2]-672*J[i-1]+    // eight-point central finite difference
                    672*J[i+1]-168*J[i+2]+ 32*J[i+3]-  3*J[i+4])/(840*T);
    }
    return J_dot;

};

};
