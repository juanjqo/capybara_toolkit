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
#include <capybara/experimental/vfi_manager.hpp>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQExperimental.h>
#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimRobotZMQ.h>
#include <capybara.hpp>

using namespace Eigen;
using namespace DQ_robotics;

namespace Capybara {

class RobotConstraintsManager: public VFI_manager
{
protected:
    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQExperimental> vi_;
    std::string config_path_;
    VFI_manager::LEVEL level_;
    std::shared_ptr<DQ_Kinematics> robot_;
    std::shared_ptr<DQ_CoppeliaSimRobot> coppelia_robot_;
    std::shared_ptr<Capybara::ConstraintsManager> CM_;
    double vfi_gain_{0.5};

public:
    RobotConstraintsManager(const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQExperimental>& coppelia_interface,
                            const std::shared_ptr<DQ_Kinematics>& robot,
                            const std::shared_ptr<DQ_CoppeliaSimRobot>& coppelia_robot,
                            const std::string &config_path,
                            const VFI_manager::LEVEL& level);

};

}





