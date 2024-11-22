#include <capybara/vfi_manager.hpp>

namespace Capybara {


VFI_manager::VFI_manager(const int &dim_configuration, const LEVEL &level)
    :dim_configuration_(dim_configuration),level_(level)
{
    constraint_manager_ = std::make_unique<Capybara::ConstraintsManager>(dim_configuration_);
}

void VFI_manager::_add_vfi_constraint(const MatrixXd &Jd, const VectorXd &b, const DIRECTION &direction)
{
    if(direction == DIRECTION::KEEP_ROBOT_OUTSIDE)
        constraint_manager_->add_inequality_constraint(-Jd, b);
    else
        constraint_manager_->add_inequality_constraint(Jd, -b);
}


void VFI_manager::add_vfi_constraint(const DIRECTION &direction,
                                     const VFI_TYPE &vfi_type,
                                     const double &safe_distance,
                                     const double &vfi_gain,
                                     const MatrixXd &robot_pose_jacobian,
                                     const DQ &robot_pose,
                                     const DQ &robot_attached_direction,
                                     const DQ &workspace_pose,
                                     const DQ &workspace_attached_direction,
                                     const DQ &workspace_derivative)
{
    switch(vfi_type)
    {

    case VFI_TYPE::RPOINT_TO_POINT:
        const double square_safe_distance = pow(safe_distance, 2);
        const DQ& x = robot_pose;
        const DQ& x_ = workspace_pose;
        const DQ p = x.translation();
        const DQ p_ = x_.translation();
        const MatrixXd Jt     = DQ_Kinematics::translation_jacobian(robot_pose_jacobian, robot_pose);
        const MatrixXd Jd     = DQ_Kinematics::point_to_point_distance_jacobian(Jt, p, p_);
        const double square_d = DQ_Geometry::point_to_point_squared_distance(p, p_);
        const double residual = DQ_Kinematics::point_to_point_residual(p, p_, workspace_derivative);
        const double square_error = square_d- square_safe_distance;
        //VectorXd b = VectorXd::Zero(1);
        //b(0)  =  vfi_gain*(square_error) + residual;
        VectorXd b = Capybara::CVectorXd({vfi_gain*(square_error) + residual});
        //std::cout<<"Distance: "<<sqrt(square_d)<<std::endl;
        _add_vfi_constraint(Jd, b, direction);
        break;
    }
}

std::tuple<MatrixXd, VectorXd> VFI_manager::get_vfi_constraints()
{
    return constraint_manager_->get_inequality_constraints();
}

} // capybara namespace
