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
#include <vector>
#include <dqrobotics/DQ.h>


using namespace Eigen;
using namespace DQ_robotics;

namespace Capybara {
class VFI_Framework
{

public:

    enum class VFI_MODE{
        ENVIRONMENT_TO_ROBOT,
        ROBOT_TO_ROBOT
    };

    enum class VFI_TYPE{
        RPOINT_TO_POINT,
        RPOINT_TO_PLANE,
        RPOINT_TO_LINE,
        RLINE_TO_LINE_ANGLE,
        RLINE_TO_LINE,
        RLINE_TO_POINT
    };
    enum class DIRECTION{
        KEEP_ROBOT_OUTSIDE,
        KEEP_ROBOT_INSIDE
    };
    enum class PRIMITIVE{
        POINT,
        LINE,
        PLANE,
        LINE_ANGLE
    };
    enum class LEVEL{
        VELOCITIES,
        ACCELERATIONS
    };

    static PRIMITIVE map_string_to_primitive  (const std::string& str);
    static DIRECTION map_string_to_direction  (const std::string& str);
    static VFI_TYPE  map_strings_to_vfiType   (const std::string& entity_robot_primitive_type,
                                               const std::string& entity_enviroment_primitive_type);
    static std::string map_vfyType_to_string   (const VFI_TYPE& vfi_type);
    static std::string map_primitive_to_string(const PRIMITIVE& primitive);
    static DQ map_attached_direction_string_to_dq(const std::string& str);
};
}
