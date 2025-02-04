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

#include "capybara/conversions.hpp"
#include <capybara/dqrobotics/rigid_motions.hpp>


namespace Capybara {

/**
 * @brief rot returns a unit quaternion that represents a desired orientation.
 * @param angle The desired rotation angle in radians.
 * @param axis  The desired rotation axis. This must be a unit pure quaternion.
 * @return The desired unit quaternion.
 */
DQ rot(const double &angle, const DQ &axis)
{
    if (!is_unit(axis) or !is_quaternion(axis) or !is_pure_quaternion(axis))
        throw std::runtime_error("Panic with Capybara::rot(angle, axis). The axis must be a unit pure quaternion.");

    return cos(angle/2)+axis*sin(angle/2);
}


/**
 * @brief rotx returns a unit quaternion that represents a desired orientation
 *              around the x-axis.
 * @param angle_deg The desired rotation angle (around the x-axis) in degrees.
 * @return The desired unit quaternion.
 */
DQ rotx(const double &angle_deg)
{
    return Capybara::rot(Capybara::Conversions::deg2rad(angle_deg), i_);
}


/**
 * @brief roty  returns a unit quaternion that represents a desired orientation
 *              around the y-axis.
 * @param angle_deg The desired rotation angle (around the y-axis) in degrees.
 * @return The desired unit quaternion.
 */
DQ roty(const double &angle_deg)
{
    return Capybara::rot(Capybara::Conversions::deg2rad(angle_deg), j_);
}

/**
 * @brief rotz returns a unit quaternion that represents a desired orientation
 *              around the z-axis.
 * @param angle_deg The desired rotation angle (around the z-axis) in degrees.
 * @return The desired unit quaternion.
 */
DQ rotz(const double &angle_deg)
{
    return Capybara::rot(Capybara::Conversions::deg2rad(angle_deg), k_);
}

/**
 * @brief translate returns a unit dual quaternion that represents the desired translation
 * @param translation The pure quaternion that represents the desired translation translation.
 * @return The unit the dual quaternion given by 1+0.5*E_*translation.
 */
DQ translate(const DQ &translation)
{
    if (!is_pure(translation) or !is_quaternion(translation))
        throw std::runtime_error("Panic with Capybara::translate(translation). The argument must be a pure quaternion.");
    return 1+0.5*E_*translation;
}


/**
 * @brief translate_xyz a unit dual quaternion that represents the desired translation
 * @param x The desired translation in the x-axis.
 * @param y The desired translation in the y-axis.
 * @param z The desired translation in the z-axis.
 * @return The unit the dual quaternion given by 1+0.5*E_*(x*i_, y*j_, z*k_).
 */
DQ translate_xyz(const double &x, const double &y, const double &z)
{
    return translate(x*i_+y*j_+z*k_);
}


/**
 * @brief translate_x returns a unit dual quaternion that represents the desired translation along the x-axis
 * @param x The desired translation in the x-axis.
 * @return The unit the dual quaternion given by 1+0.5*E_*(x*i_)
 */
DQ translate_x(const double &x)
{
    return translate(x*i_);
}


/**
 * @brief translate_y returns unit dual quaternion that represents the desired translation along the y-axis
 * @param y The desired translation in the y-axis.
 * @return The unit the dual quaternion given by 1+0.5*E_*(y*j_)
 */
DQ translate_y(const double &y)
{
    return translate(y*j_);
}


/**
 * @brief translate_z returns unit dual quaternion that represents the desired translation along the z-axis
 * @param z The desired translation in the z-axis.
 * @return The unit the dual quaternion given by 1+0.5*E_*(z*k_)
 */
DQ translate_z(const double &z)
{
    return translate(z*k_);
}

}
