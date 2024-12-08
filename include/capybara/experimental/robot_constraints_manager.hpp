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
#include <yaml-cpp/yaml.h>

using namespace Eigen;
using namespace DQ_robotics;

namespace Capybara {

class RobotConstraintsManager
{
protected:
    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQExperimental> vi_;
    std::string config_path_;
    VFI_manager::LEVEL level_;
    std::shared_ptr<DQ_Kinematics> robot_;
    std::shared_ptr<DQ_CoppeliaSimRobot> coppelia_robot_;
    std::shared_ptr<Capybara::VFI_manager> VFI_M_;
    double vfi_gain_{0.5};

    VectorXd q_max_;
    VectorXd q_min_;
    VectorXd q_min_dot_;
    VectorXd q_max_dot_;


    std::vector<VFI_manager::VFI_MODE> vfi_mode_list_;


    std::vector<VFI_manager::VFI_TYPE> vfi_type_list_;
    std::vector<VFI_manager::DIRECTION> direction_list_;
    std::vector<double> safe_distance_list_;
    std::vector<int> robot_index_list_;
    std::vector<int> joint_index_list_;
    std::vector<DQ> dq_offset_list_;
    std::vector<DQ> robot_attached_dir_list_;
    std::vector<DQ> envir_attached_dir_list_;
    std::vector<DQ> workspace_derivative_list_;
    std::vector<std::string> cs_entity_environment_list_;

    DQ _get_robot_primitive_offset_from_coppeliasim(const std::string& object_name, const int& joint_index);
    void _initial_settings();
public:
    RobotConstraintsManager(const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQExperimental>& coppelia_interface,
                            const std::shared_ptr<DQ_Kinematics>& robot,
                            const std::shared_ptr<DQ_CoppeliaSimRobot>& coppelia_robot,
                            const VectorXd& q_min,
                            const VectorXd& q_max,
                            const VectorXd& q_min_dot,
                            const VectorXd& q_max_dot,
                            const std::string &config_path,
                            const VFI_manager::LEVEL& level = VFI_manager::LEVEL::VELOCITIES);

    std::tuple<MatrixXd, VectorXd> get_inequality_constraints(const VectorXd& q);
    std::tuple<MatrixXd, VectorXd> get_equality_constraints();


};

}





