#include <capybara/dqrobotics/custom_controllers.hpp>






Capybara::CustomControllers::CustomControllers(const TYPE &type, const std::shared_ptr<DQ_QuadraticProgrammingSolver> &solver)
    :type_{type}, solver_{solver}
{

}

void Capybara::CustomControllers::set_proportional_gain(const double &gain)
{
    gain_ = gain;
}

void Capybara::CustomControllers::set_damping(const double &damping)
{
    damping_ = damping;
}

/**
 * @brief Capybara::CustomControllers::set_alpha sets the priority of the tasks.
 * @param alpha A scalar between 0 and 1. When alpha = 1 the task is a translation task only. When alpha = 0 the task is a rotation task only.
 *        Other values represent a linear combination between both tasks.
 */
void Capybara::CustomControllers::set_alpha(const double &alpha)
{
    if (alpha > 1 or alpha < 0)
        throw std::runtime_error(std::string("Error in Capybara::CustomControllers::set_alpha.  alpha must a double within the interval [0,1]"));
    alpha_ = alpha;
}

void Capybara::CustomControllers::set_region_size(const double &region_size)
{
    region_size_ = region_size;
}

void Capybara::CustomControllers::set_region_exit_size(const double &region_exit_size)
{
    region_exit_size_ = region_exit_size;
}

/**
 * @brief Capybara::CustomControllers::get_rotation_error
 * @param x
 * @param xd
 * @return
 */
VectorXd Capybara::CustomControllers::_get_rotation_error(const DQ &x, const DQ &xd)
{
    VectorXd error_1 =  vec4( x.rotation().conj()*xd.rotation() - 1 );
    VectorXd error_2 =  vec4( x.rotation().conj()*xd.rotation() + 1 );

    double norm_1 = error_1.norm();
    double norm_2 = error_2.norm();

    if (norm_1 < norm_2){
        return error_1;
    }else{
        return error_2;
    }
}

/**
 * @brief Capybara::CustomControllers::compute_setpoint_control_signal
 * @param x
 * @param xd
 * @param pose_jacobian
 * @param inequality_constraints
 * @param equality_constraints
 * @param solver
 * @return
 */
VectorXd Capybara::CustomControllers::compute_setpoint_control_signal(const DQ &x,
                                                                      const DQ &xd,
                                                                      const MatrixXd& pose_jacobian,
                                                                      const std::tuple<MatrixXd, VectorXd>& inequality_constraints,
                                                                      const std::tuple<MatrixXd, VectorXd> &equality_constraints)
{
    switch (type_) {

    case TYPE::POSITION_AND_ORIENTATION_COMBINATION:
        return _compute_setpoint_using_POSITION_AND_ORIENTATION_COMBINATION(x, xd, pose_jacobian, inequality_constraints, equality_constraints);
        break;
    default:
        throw std::runtime_error(std::string("Error in Capybara::CustomControllers::compute_setpoint_control_signal.  Unsupported controller"));
        break;
    }
}

/**
 * @brief Capybara::CustomControllers::_compute_setpoint_using_POSITION_AND_ORIENTATION_COMBINATION computes the control inputs
 *           using the a control law based on the following paper
 *
 *
 *           M. M. Marinho et al., "A Unified Framework for the Teleoperation of Surgical Robots in Constrained Workspaces,"
 *           2019 International Conference on Robotics and Automation (ICRA), Montreal, QC, Canada, 2019, pp. 2721-2727,
 *           doi: 10.1109/ICRA.2019.8794363.
 *
 *
 *
 * @param x
 * @param xd
 * @param pose_jacobian
 * @param inequality_constraints
 * @param equality_constraints
 * @return
 */
VectorXd Capybara::CustomControllers::_compute_setpoint_using_POSITION_AND_ORIENTATION_COMBINATION(const DQ &x, const DQ &xd, const MatrixXd &pose_jacobian, const std::tuple<MatrixXd, VectorXd> &inequality_constraints, const std::tuple<MatrixXd, VectorXd> &equality_constraints)
{
    VectorXd u;
    VectorXd et = vec4(x.translation() - xd.translation());

    double d = (x.translation()-xd.translation()).vec3().norm();
    if (d <region_size_ and !robot_reached_region)
    {
        et = VectorXd::Zero(4);
    }

    if (d > region_exit_size_ )
    {
        robot_reached_region = false;
    }



    VectorXd er = _get_rotation_error(x, xd);

    const MatrixXd& Jx = pose_jacobian;
    DQ rd = xd.rotation();
    MatrixXd Jr = DQ_Kinematics::rotation_jacobian(Jx);
    MatrixXd Nr = haminus4(rd)*C4()*Jr;
    MatrixXd Jt = DQ_Kinematics::translation_jacobian(Jx, x);



    MatrixXd Ht = Jt.transpose()*Jt;
    VectorXd ft = gain_*Jt.transpose()*et;

    MatrixXd Hr = Nr.transpose()*Nr;
    VectorXd fr = gain_*Nr.transpose()*er;

    MatrixXd Hd = MatrixXd::Identity(Ht.cols(), Ht.cols())*damping_*damping_;

    MatrixXd H = alpha_*Ht + (1.0 - alpha_)*Hr + Hd;
    VectorXd f = alpha_*ft + (1.0 - alpha_)*fr;

    auto A = std::get<0>(inequality_constraints);
    auto b = std::get<1>(inequality_constraints);
    auto Aeq = std::get<0>(equality_constraints);
    auto beq = std::get<1>(equality_constraints);

    u = solver_->solve_quadratic_program(H,
                                         f,
                                         std::get<0>(inequality_constraints),
                                         std::get<1>(inequality_constraints),
                                         std::get<0>(equality_constraints),
                                         std::get<1>(equality_constraints));
    /*
    try {
        u = solver_->solve_quadratic_program(H,
                                                      f,
                                                      std::get<0>(inequality_constraints),
                                                      std::get<1>(inequality_constraints),
                                                      std::get<0>(equality_constraints),
                                                      std::get<1>(equality_constraints));
    } catch (...) {
        u = VectorXd::Zero(pose_jacobian.cols());
        std::cerr<<"Capybara::CustomControllers::Failed to solve the optimization problem!"<<std::endl;
    }
*/


    return u;
}
