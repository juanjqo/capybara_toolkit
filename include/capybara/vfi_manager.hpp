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

using namespace Eigen;
using namespace DQ_robotics;

namespace Capybara {

class VFI_manager
{
public:
    enum class VFI_TYPE{
        RPOINT_TO_POINT,
        //RPOINT_TO_LINE,
        //RPOINT_TO_PLANE,
        //RLINE_TO_LINE_ANGLE,
        //RLINE_TO_LINE,
        //RLINE_TO_POINT
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

protected:
    int dim_configuration_;
    LEVEL level_;
    std::unique_ptr<Capybara::ConstraintsManager> constraint_manager_;
    void _add_vfi_constraint(const MatrixXd& Jd,
                             const VectorXd& b,
                             const DIRECTION& direction);
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
                            const DQ& workspace_derivative);

    //void add_sovfi_constraint();

    std::tuple<MatrixXd, VectorXd> get_vfi_constraints();


};
}

