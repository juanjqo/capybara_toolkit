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
#include <dqrobotics/DQ.h>
#include <capybara/conversions.hpp>

using namespace  DQ_robotics;
using namespace Eigen;

namespace Capybara {

    DQ rot(const double& angle, const DQ& axis);
    DQ rotx(const double& angle_deg);
    DQ roty(const double& angle_deg);
    DQ rotz(const double& angle_deg);
    DQ translate(const DQ& translation);
    DQ translate_xyz(const double& x, const double& y, const double& z);
    DQ translate_x(const double& x);
    DQ translate_y(const double& y);
    DQ translate_z(const double& z);
}

