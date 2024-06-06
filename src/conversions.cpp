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

#include <capybara/conversions.hpp>

namespace Capybara {

/**
 * @brief Capybara::Conversions::std_vector_double_to_vectorxd
 * @param std_vector
 * @return
 */
Eigen::VectorXd Capybara::Conversions::std_vector_double_to_vectorxd(std::vector<double> &std_vector)
{
    return Eigen::Map<VectorXd>(std_vector.data(), std_vector.size());
}

/**
 * @brief Conversions::std_vector_vectorxd_to_vectorxd
 * @param std_vectorxd
 * @return
 */
VectorXd Conversions::std_vector_vectorxd_to_vectorxd(std::vector<VectorXd> &std_vectorxd)
{
    VectorXd q;
    for (auto &vec : std_vectorxd)
    {
        q = Capybara::Numpy::vstack(q, vec);
    }
    return q;
}

/**
 * @brief Conversions::double2vector
 * @param value
 * @param size
 * @return
 */
VectorXd Conversions::double2vector(const double &value, const int &size)
{
    VectorXd output = VectorXd(size);
    for (auto i = 0; i<size; i++)
    {
        output(i) = value;
    }
    return output;
}

/**
 * @brief Conversions::rad2deg
 * @param rad
 * @return
 */
double Conversions::rad2deg(const double &rad)
{
    return rad*(180/Capybara::pi);
}

/**
 * @brief Conversions::rad2deg
 * @param rad
 * @return
 */
VectorXd Conversions::rad2deg(const VectorXd &rad)
{
    VectorXd output = rad;
    for (size_t i = 0; auto& value : rad)
    {
        output(i) = rad2deg(value);
        ++i;
    }
    return output;
}

/**
 * @brief Conversions::deg2rad
 * @param deg
 * @return
 */
double Conversions::deg2rad(const double &deg)
{
    return deg*(Capybara::pi/180);
}

/**
 * @brief Conversions::deg2rad
 * @param deg
 * @return
 */
VectorXd Conversions::deg2rad(const VectorXd &deg)
{
    VectorXd output = deg;
    for (size_t i = 0; auto& value : deg)
    {
        output(i) = deg2rad(value);
        ++i;
    }
    return output;
}

}
