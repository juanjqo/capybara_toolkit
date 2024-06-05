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

#pragma once
#include <capybara/checkers.hpp>
#include <capybara/conversions.hpp>

#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

using namespace Eigen;

namespace Capybara {

class Numpy
{
public:
    static MatrixXd vstack(const MatrixXd& A, const MatrixXd& B);
    static MatrixXd hstack(const MatrixXd& A, const MatrixXd& B);
    static MatrixXd block_diag(const std::vector<MatrixXd>& A);
    static MatrixXd block_diag(const std::vector<double>& A);
    static MatrixXd resize(const MatrixXd& A, const int& rows, const int& cols);
    static VectorXd linspace(const double& start, const double& stop, const int& size);
    static MatrixXd linspace(const VectorXd& start, const VectorXd& stop, const int& size);

};
}


