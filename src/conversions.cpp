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
 * @brief Capybara::Conversions::std_vector_double_to_vectorxd converts a vector to an Eigen vector
 * @param std_vector
 * @return The desired Eigen vector.
 */
Eigen::VectorXd Capybara::Conversions::std_vector_double_to_vectorxd(std::vector<double> &std_vector)
{
    return Eigen::Map<VectorXd>(std_vector.data(), std_vector.size());
}

/**
 * @brief Conversions::std_vector_vectorxd_to_vectorxd converts a vector of vectors to an Eigen vector
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
 * @brief Conversions::double2vector creates a vector of the specified size with all their elements
 *                                   initialized in a specific value.
 * @param value The value to initialize all vector elements.
 * @param size The desired size of the vector
 * @return The desired Eigen vector.
 */
VectorXd Conversions::double2vector(const double &value, const int &size)
{
    std::vector<double> aux(size, value);
    return std_vector_double_to_vectorxd(aux);
}

/**
 * @brief Conversions::rad2deg converts radians to degrees.
 * @param rad The radians to be converted to degrees
 * @return The desired degrees
 */
double Conversions::rad2deg(const double &rad)
{
    return rad*(180/Capybara::pi);
}

/**
 * @brief Conversions::rad2deg converts an Eigen vector of radians to an Eigen vector containing degrees.
 * @param rad The input Eigen vector that containts the radians
 * @return The desired Eigen vector of degrees
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
 * @brief Conversions::deg2rad converts degrees to radians.
 * @param deg The degrees to be converted to radians.
 * @return The desired radians
 */
double Conversions::deg2rad(const double &deg)
{
    return deg*(Capybara::pi/180);
}

/**
 * @brief Conversions::deg2rad converts an Eigen vector of degrees to an Eigen vector containing radians.
 * @param deg The input Eigen vector that contatins the degrees
 * @return The desired Eigen vector of radians
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
