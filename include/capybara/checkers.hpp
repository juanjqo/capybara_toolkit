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
#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif


using namespace Eigen;

namespace Capybara {

class Checkers
{
public:
    enum class MODE{PANIC, DO_NOT_PANIC};
    static bool check_column_matrix_sizes(const MatrixXd& A,
                                          const MatrixXd& B,
                                          const Checkers::MODE& mode = Checkers::MODE::PANIC);

    static bool check_row_matrix_sizes(const MatrixXd& A,
                                       const MatrixXd& B,
                                       const MODE& mode = Checkers::MODE::PANIC);


    static bool check_constraint_sizes(const MatrixXd& A,
                                       const VectorXd& b,
                                       const double& optimization_vector_size,
                                       const MODE& mode = Checkers::MODE::PANIC);

    static bool is_string(const auto& input){
        return typeid(input) == typeid(std::string(""));
    }

    //template<typename T> // For old C++ versions
    static bool check_equal_elements(const auto& vector,
                                     const MODE& mode = Checkers::MODE::PANIC)
    {
        if (std::all_of(vector.cbegin(), vector.cend(), [vector](int i) { return i == vector.at(0); }))
        {
            return true;
        }else{
            if (mode == Checkers::MODE::PANIC)
            {
                throw std::runtime_error("Panic with Capybara::check_for_equal_elements(vector). The vector containts different elements. ");
            }
            return false;
        }
    }

    //template<typename T, typename U>  // For old C++ versions
    static bool check_equal_sizes(const auto &v1,
                                  const auto &v2,
                                  const MODE& mode = Checkers::MODE::PANIC)
    {
        std::size_t s1 = static_cast<std::size_t>(v1.size());
        std::size_t s2 = static_cast<std::size_t>(v2.size());
        if (s1 != s2)
        {
            if (mode == Checkers::MODE::PANIC)
                throw std::runtime_error("Panic with Capybara::Checkers::check_equal_sizes(v1, v2). Both containers have diferent sizes. ");
            return FAIL;
        }
        return SUCCESS;
    }


    //template<typename T, typename U, typename V> // For old C++ versions
    static bool check_equal_sizes(const auto &v1,
                                  const auto &v2,
                                  const auto &v3,
                                  const MODE& mode = Checkers::MODE::PANIC)
    {
        bool s1 = Checkers::check_equal_sizes(v1, v2, mode);
        bool s2 = Checkers::check_equal_sizes(v2, v3, mode);
        if (s1 != s2)
        {
            return FAIL;
        }
        return SUCCESS;
    }





};


}


