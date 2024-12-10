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
#ifdef _WIN32
#include <Eigen/Dense>
#else
#include <eigen3/Eigen/Dense>
#endif

#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <dqrobotics/utils/DQ_Geometry.h>
#include <capybara.hpp>
#include <capybara/experimental/vfi_framework.hpp>
#include <memory>

using namespace Eigen;
using namespace DQ_robotics;

namespace Capybara {

class VFI_manager: public VFI_Framework
{


protected:
    int dim_configuration_;
    LEVEL level_;
    std::shared_ptr<Capybara::ConstraintsManager> constraint_manager_;
    void _add_vfi_constraint(const MatrixXd& Jd,
                             const VectorXd& b,
                             const DIRECTION& direction);

    VectorXd q_dot_min_ = VectorXd::Zero(0);
    VectorXd q_dot_max_ = VectorXd::Zero(0);
    VectorXd q_min_ = VectorXd::Zero(0);
    VectorXd q_max_ = VectorXd::Zero(0);


public:
    VFI_manager()=delete;
    VFI_manager(const int& dim_configuration,
                const LEVEL& level = LEVEL::VELOCITIES);

    void add_vfi_constraint(const DIRECTION& direction,
                            const VFI_TYPE& vfi_type,
                            const double& safe_distance,
                            const double& vfi_gain,
                            const MatrixXd &robot_pose_jacobian,
                            const DQ& robot_pose,
                            const DQ& robot_attached_direction,
                            const DQ& workspace_pose,
                            const DQ& workspace_attached_direction,
                            const DQ& workspace_derivative = DQ(0));

    void set_joint_position_limits(const VectorXd& q_lower_bound, const VectorXd& q_upper_bound);
    void set_joint_velocity_limits(const VectorXd& q_dot_lower_bound, const VectorXd& q_dot_upper_bound);
    //void add_sovfi_constraint();




    std::tuple<MatrixXd, VectorXd> get_inequality_constraints();
    std::tuple<MatrixXd, VectorXd> get_equality_constraints();


};
}

