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
    {
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
        VectorXd b = Capybara::CVectorXd({vfi_gain*(square_error) + residual});
        _add_vfi_constraint(Jd, b, direction);
        break;
    }
    case VFI_TYPE::RPOINT_TO_PLANE:
    {
        const DQ p = robot_pose.translation();
        const DQ& x_ = workspace_pose;
        const DQ plane_normal = x_.P()*workspace_attached_direction*x_.P().conj();
        const DQ plane_point =  x_.translation();
        const DQ workspace_plane  = plane_normal + E_*dot(plane_point, plane_normal);
        const MatrixXd Jt     =  DQ_Kinematics::translation_jacobian(robot_pose_jacobian, robot_pose);
        const MatrixXd Jd     =  DQ_Kinematics::point_to_plane_distance_jacobian(Jt, p, workspace_plane);
        const double residual =  DQ_Kinematics::point_to_plane_residual(p, workspace_derivative);
        const double d = DQ_Geometry::point_to_plane_distance(p, workspace_plane);
        const double error = d - safe_distance;
        VectorXd b = Capybara::CVectorXd({vfi_gain*(error) + residual});
        _add_vfi_constraint(Jd, b, direction);
        break;
    }

    case VFI_TYPE::RPOINT_TO_LINE:
    {
        const DQ& x= workspace_pose;
        const DQ l_= (x.P())*workspace_attached_direction*(x.P().conj());
        const DQ p_= x.translation();
        const DQ workspace_line = l_ + E_*cross(p_, l_);
        const double square_safe_distance = pow(safe_distance, 2);
        const DQ p = robot_pose.translation();
        const MatrixXd Jt =  DQ_Kinematics::translation_jacobian(robot_pose_jacobian, robot_pose);
        const MatrixXd Jd = DQ_Kinematics::point_to_line_distance_jacobian(Jt, p, workspace_line);
        const double residual = DQ_Kinematics::point_to_line_residual(p, workspace_line, workspace_derivative);
        const double square_d = DQ_Geometry::point_to_line_squared_distance(p, workspace_line);
        const double square_error = square_d - square_safe_distance;
        VectorXd b = Capybara::CVectorXd({vfi_gain*(square_error) + residual});
        _add_vfi_constraint(Jd, b, direction);
        break;
    }
    case VFI_TYPE::RLINE_TO_LINE_ANGLE:
    {
        const DQ workspace_line = (workspace_pose.P())*workspace_attached_direction*(workspace_pose.P().conj());
        const double safe_angle = safe_distance*(pi/180);  //Convert to radians
        const DQ& robot_line_direction = robot_attached_direction;
        const MatrixXd Jl = DQ_Kinematics::line_jacobian(robot_pose_jacobian, robot_pose,robot_line_direction);
        const DQ r = robot_pose.P();
        const DQ robot_line = r*(robot_line_direction)*r.conj();
        const MatrixXd Jfphi = DQ_Kinematics::line_to_line_angle_jacobian(Jl,robot_line,workspace_line);
        const double fsafe = 2-2*cos(safe_angle);
        const double phi = DQ_Geometry::line_to_line_angle(robot_line, workspace_line);
        const double f = 2-2*cos(phi);
        const double ferror = f-fsafe;
        const double residual = DQ_Kinematics::line_to_line_angle_residual(robot_line,workspace_line,-workspace_derivative);
        VectorXd b = Capybara::CVectorXd({vfi_gain*(ferror) + residual});
        _add_vfi_constraint(Jfphi, b, direction);
        break;
    }
}
}

std::tuple<MatrixXd, VectorXd> VFI_manager::get_vfi_constraints()
{
    return constraint_manager_->get_inequality_constraints();
}

} // capybara namespace
