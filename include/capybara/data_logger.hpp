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
#include <capybara/capytypes.hpp>
#include <capybara/numpy.hpp>
#include <capybara/conversions.hpp>

#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif
#include <fstream>
#include <iostream>
using namespace Eigen;

namespace Capybara {
class DataLogger
{
public:
    enum class TYPE{
        VECTORXD
    };
private:
    TYPE type_;
    std::ofstream data_logger_;
    bool first_call_{true};
    MatrixXd matrix_data_;
    int vector_size_;
    int i_{0};
    //list_vel_ref.open("list_vel_ref.csv");
public:
    DataLogger(const TYPE& type);


    void add_data(const VectorXd& data);
    void save_data(const std::string& filename);
    void show_data();
};
}

