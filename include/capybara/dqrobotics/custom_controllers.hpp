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
#include<dqrobotics/robot_modeling/DQ_Kinematics.h>
#include <capybara.hpp>
#include <dqrobotics/solvers/DQ_QuadraticProgrammingSolver.h>
#include <memory>

using namespace  DQ_robotics;
using namespace Eigen;

namespace Capybara {


class CustomControllers
{
public:
    enum class TYPE{
        POSITION_AND_ORIENTATION_COMBINATION
    };

protected:
    TYPE type_;
    std::shared_ptr<DQ_QuadraticProgrammingSolver> solver_;

    double gain_   {0.1};
    double damping_{0.05};
    double alpha_  {0.99};
    double region_size_{0.5};
    double region_exit_size_{0.55};
    bool robot_reached_region{false};

    VectorXd _get_rotation_error(const DQ& x, const DQ& xd);

    VectorXd _compute_setpoint_using_POSITION_AND_ORIENTATION_COMBINATION(const DQ& x,
                                                                         const DQ& xd,
                                                                         const MatrixXd& pose_jacobian,
                                                                         const std::tuple<MatrixXd, VectorXd>& inequality_constraints,
                                                                          const std::tuple<MatrixXd, VectorXd>& equality_constraints);

public:
    CustomControllers(const TYPE& type, const std::shared_ptr<DQ_QuadraticProgrammingSolver>& solver);
    void set_proportional_gain(const double& gain);
    void set_damping(const double& damping);
    void set_alpha(const double& alpha);
    void set_region_size(const double& region_size);
    void set_region_exit_size(const double& region_exit_size);
    VectorXd compute_setpoint_control_signal(const DQ& x,
                                             const DQ& xd,
                                             const MatrixXd& pose_jacobian,
                                             const std::tuple<MatrixXd, VectorXd>& inequality_constraints,
                                             const std::tuple<MatrixXd, VectorXd>& equality_constraints);
};
}




